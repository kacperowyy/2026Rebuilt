// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands_timed.modules.intake.*;
import frc.robot.commands_timed.modules.shooting.*;

import frc.robot.commands.modules.intake.IntakeOnCommand;
import frc.robot.commands.modules.intake.IntakeDropCloseCommand;
import frc.robot.commands.modules.intake.IntakeDropCommand;
import frc.robot.commands.modules.intake.IntakeReverseCommand;
import frc.robot.commands.modules.shooting.Shoot;
import frc.robot.commands.modules.shooting.SortAndPass;
import frc.robot.commands.modules.shooting.SortAndPassReverse;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeDrop;
import frc.robot.subsystems.Shooting;
import frc.robot.subsystems.Vision.Aim;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        private final SendableChooser<Command> autoChooser = new SendableChooser<>();
        private final Command doNothingAuto = Commands.none();
        private final String defaultAutoName;

        final CommandXboxController supportXbox = new CommandXboxController(OperatorConstants.kSupportControllerPort);
        final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));

        private final DoubleSupplier driverXboxRightXInverted = () -> -driverXbox.getRightX();

        private final Intake intake = new Intake();
        private final IntakeDrop intakeDrop = new IntakeDrop();
        private final Shooting shooting = new Shooting();
        private final Aim aim = new Aim();
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverXbox.getLeftY() * 1,
                        () -> -driverXbox.getLeftX() * 1)
                        .withControllerRotationAxis(driverXboxRightXInverted)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(false);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        private final SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY).headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        private final SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("Shoot", new ShootTimed(shooting, 2.0));
                NamedCommands.registerCommand("SortAndPass", new SortAndPassTimed(shooting, 2.0));
                NamedCommands.registerCommand("IntakeOn", new IntakeOnCommandTimed(intake, 2.0));
                NamedCommands.registerCommand("IntakeDrop", new IntakeDropCommandTimed(intakeDrop, 2.0));

                defaultAutoName = configureAutoChooser();
                SmartDashboard.putData("Select Autonomous", autoChooser);
                SmartDashboard.putData("Auto choices", autoChooser);
                SmartDashboard.putData("Autonomous Mode", autoChooser);
                SmartDashboard.putData("Autonomous", autoChooser);
        }


        public SwerveSubsystem getSwerve() {
    return drivebase;
}

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureBindings() {
                Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

                // Zero gyroscope
                driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

                // Placeholders
                driverXbox.start().whileTrue(Commands.none());
                driverXbox.back().whileTrue(Commands.none());

                // Hold right bumper for intake
                driverXbox.rightBumper().whileTrue(new IntakeOnCommand(intake));
                // Hold left bumper for intake reverse
                driverXbox.leftBumper().whileTrue(new IntakeReverseCommand(intake));

                // Hold right trigger for intake drop
                driverXbox.x().whileTrue(new IntakeDropCommand(intakeDrop));
                driverXbox.povLeft().whileTrue(new IntakeDropCloseCommand(intakeDrop));

                // Shooting command with driver translation + automatic tower facing
                driverXbox.b().onTrue(Commands.runOnce(drivebase::cancelAimAndDrive));
                driverXbox.b().whileTrue(
                        Commands.runEnd(
                                () -> {
                                        shooting.startShooting();
                                        drivebase.driveAndFaceTower(-driverXbox.getLeftY(), -driverXbox.getLeftX());
                                },
                                shooting::stop,
                                shooting,
                                drivebase));
                driverXbox.povLeft().whileTrue(new IntakeDropCloseCommand(intakeDrop));

                // Shooting commands without shooting (pov = dpad btw)
                supportXbox.povUp().whileTrue(new SortAndPass(shooting));
                supportXbox.povDown().whileTrue(new SortAndPassReverse(shooting));
                
                //drive to pose
                driverXbox.y()
                                .and(driverXbox.b().negate())
                                .onTrue(Commands.runOnce(drivebase::startAimAndDrive));
                driverXbox.povRight().onTrue(Commands.runOnce(this::cancelDrivebaseCurrentCommand));
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        private String configureAutoChooser() {
                List<String> autoNames = discoverAutoNames();
                autoChooser.setDefaultOption("Do Nothing", doNothingAuto);

                if (autoNames.isEmpty()) {
                        SmartDashboard.putStringArray("Auto List", new String[] { "Do Nothing" });
                        SmartDashboard.putString("Auto Selector", "Do Nothing");
                        SmartDashboard.putString("Auto Chooser Status", "No autos found in deploy/pathplanner/autos");
                        return "";
                }

                String chosenDefault = pickDefaultAuto(autoNames);
                int loadedAutos = 0;

                for (String autoName : autoNames) {
                        Command autoCommand = loadAutoCommandByName(autoName);
                        if (autoCommand == null) {
                                continue;
                        }

                        loadedAutos++;
                        if (autoName.equals(chosenDefault)) {
                                autoChooser.setDefaultOption(autoName + " (Default)", autoCommand);
                        } else {
                                autoChooser.addOption(autoName, autoCommand);
                        }
                }

                SmartDashboard.putStringArray("Auto List", autoNames.toArray(String[]::new));
                SmartDashboard.putString("Auto Selector", chosenDefault);
                SmartDashboard.putString("Auto Chooser Default", chosenDefault);
                SmartDashboard.putString("Auto Chooser Status", "Loaded " + loadedAutos + " autos");
                return chosenDefault;
        }

        private List<String> discoverAutoNames() {
                File autosDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
                File[] autoFiles = autosDirectory.listFiles((directory, fileName) -> fileName.endsWith(".auto"));
                if (autoFiles == null || autoFiles.length == 0) {
                        return List.of();
                }

                return Arrays.stream(autoFiles)
                                .map(File::getName)
                                .map(fileName -> fileName.substring(0, fileName.length() - ".auto".length()))
                                .sorted(String.CASE_INSENSITIVE_ORDER)
                                .toList();
        }

        private String pickDefaultAuto(List<String> autoNames) {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        String allianceDefault = alliance.get() == DriverStation.Alliance.Red ? "Red middle" : "Blue middle";
                        if (autoNames.contains(allianceDefault)) {
                                return allianceDefault;
                        }
                }

                if (autoNames.contains("Blue middle")) {
                        return "Blue middle";
                }
                return autoNames.get(0);
        }

        private Command loadAutoCommandByName(String autoName) {
                try {
                        return new PathPlannerAuto(autoName);
                } catch (Exception ex) {
                        DriverStation.reportWarning("Skipping invalid auto '" + autoName + "'", ex.getStackTrace());
                        return null;
                }
        }

        private String getDashboardSelectedAutoName() {
                String selectedByDashboard = SmartDashboard.getString("Auto Selector", defaultAutoName);
                if (selectedByDashboard == null || selectedByDashboard.isBlank()) {
                        return defaultAutoName;
                }
                return selectedByDashboard.trim();
        }

        private Command getFallbackAutoCommand() {
                List<String> autoNames = discoverAutoNames();
                if (autoNames.isEmpty()) {
                        return doNothingAuto;
                }

                String fallbackName = pickDefaultAuto(autoNames);
                Command fallbackCommand = loadAutoCommandByName(fallbackName);
                return fallbackCommand != null ? fallbackCommand : doNothingAuto;
        }

        private void cancelDrivebaseCurrentCommand() {
                Command currentDriveCommand = drivebase.getCurrentCommand();
                if (currentDriveCommand != null) {
                        currentDriveCommand.cancel();
                }
        }

        public Command getAutonomousCommand() {
                String dashboardSelection = getDashboardSelectedAutoName();
                if (dashboardSelection != null
                                && !dashboardSelection.isBlank()
                                && !"Do Nothing".equalsIgnoreCase(dashboardSelection)) {
                        Command dashboardAuto = loadAutoCommandByName(dashboardSelection);
                        if (dashboardAuto != null) {
                                SmartDashboard.putString("Selected Auto", dashboardSelection + " (from Auto Selector)");
                                return dashboardAuto;
                        }
                }

                Command chooserAuto = autoChooser.getSelected();
                if (chooserAuto != null) {
                        SmartDashboard.putString("Selected Auto", chooserAuto.getName());
                        return chooserAuto;
                }

                Command fallbackAuto = getFallbackAutoCommand();
                SmartDashboard.putString("Selected Auto", fallbackAuto.getName());
                return fallbackAuto;
        }
}

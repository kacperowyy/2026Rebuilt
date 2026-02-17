// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.modules.shooting.Shoot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.List;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Shooting;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.

        
        final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));

        DoubleSupplier driverXboxRightXInverted = () -> -new XboxController(OperatorConstants.kDriverControllerPort).getRightX(); 

        private final Shooting shooting = new Shooting();

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * 1,
                        () -> driverXbox.getLeftX() * 1)
                        .withControllerRotationAxis(driverXboxRightXInverted)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY).headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
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
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                // Zero gyroscope
                driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

                // Placeholders
                driverXbox.start().whileTrue(Commands.none());
                driverXbox.back().whileTrue(Commands.none());
                driverXbox.rightBumper().onTrue(Commands.none());
                
                // Shooting command
                driverXbox.rightTrigger(0.4).whileTrue(new Shoot(shooting));

                //drive to pose
                driverXbox.y().onTrue(drivebase.driveToClosestPose());
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

          public Command getAutonomousCommand() {

                try{
                        // Load the path you want to follow using its name in the GUI
                        // Create a list of waypoints from poses. Each pose represents one waypoint.
                        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
                        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0))
                        );

                        PathConstraints constraints = new PathConstraints(1.0, 0.5, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
                        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

                        // Create the path using the waypoints created above
                        PathPlannerPath path = new PathPlannerPath(
                                waypoints,
                                constraints,
                                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                                new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
                        );

                        // Prevent the path from being flipped if the coordinates are already correct
                        path.preventFlipping = true;

                        // Create a path following command using AutoBuilder. This will also trigger event markers.
                        return AutoBuilder.followPath(path);
                } catch (Exception e) {
                        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                        return Commands.none();
                }
        }
}

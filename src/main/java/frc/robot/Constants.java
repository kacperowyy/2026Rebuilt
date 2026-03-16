// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.Shooting;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static void initOptions() {    
    // Intake
    SmartDashboard.putNumber("Constants/Intake/PercentOutput", IntakeConstants.kPercentOutputIntake);
    SmartDashboard.putNumber("Constants/Intake/Accel", IntakeConstants.kIntakeAccel);

    // Intake Drop
    SmartDashboard.putNumber("Constants/IntakeDrop/PercentOutput", IntakeDropConstants.kPercentOutputIntakeDrop);
    
    // Shooting
    SmartDashboard.putNumber("Constants/Shooting/PercentOutputSorting", ShootingConstants.kPercentOutputSorting);
    SmartDashboard.putNumber("Constants/Shooting/PercentOutputPassthrough", ShootingConstants.kPercentOutputPassthrough);
    SmartDashboard.putNumber("Constants/Shooting/PercentOutputShooter", ShootingConstants.kPercentOutputShooter);
    SmartDashboard.putNumber("Constants/Shooting/ShooterAccel", ShootingConstants.kShooterAccel);
    SmartDashboard.putNumber("Constants/Shooting/ReferenceDistanceMeters", ShootingConstants.kShooterReferenceDistanceMeters);
    SmartDashboard.putNumber("Constants/Shooting/PercentPerMeter", ShootingConstants.kShooterPercentPerMeter);

    // Auto
    SmartDashboard.putNumber("Constants/Auto/MaxSpeedMetersPerSecond", AutoConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("Constants/Auto/MaxAccelerationMetersPerSecondSquared", AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    SmartDashboard.putNumber("Constants/Auto/MaxAngularSpeedRadiansPerSecond", AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    SmartDashboard.putNumber("Constants/Auto/MaxAngularSpeedRadiansPerSecondSquared", AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static void refresh() {
    // Intake
    IntakeConstants.kPercentOutputIntake = SmartDashboard.getNumber("Constants/Intake/PercentOutput", IntakeConstants.kPercentOutputIntake);
    IntakeConstants.kIntakeAccel = SmartDashboard.getNumber("Constants/Intake/Accel", IntakeConstants.kIntakeAccel);
    Intake.updateRateLimiter();

    // Intake Drop
    IntakeDropConstants.kPercentOutputIntakeDrop = SmartDashboard.getNumber("Constants/IntakeDrop/PercentOutput", IntakeDropConstants.kPercentOutputIntakeDrop);
    
    // Shooting
    ShootingConstants.kPercentOutputSorting = SmartDashboard.getNumber("Constants/Shooting/PercentOutputSorting", ShootingConstants.kPercentOutputSorting);
    ShootingConstants.kPercentOutputPassthrough = SmartDashboard.getNumber("Constants/Shooting/PercentOutputPassthrough", ShootingConstants.kPercentOutputPassthrough);
    ShootingConstants.kPercentOutputShooter = SmartDashboard.getNumber("Constants/Shooting/PercentOutputShooter", ShootingConstants.kPercentOutputShooter);
    ShootingConstants.kShooterAccel = SmartDashboard.getNumber("Constants/Shooting/ShooterAccel", ShootingConstants.kShooterAccel);
    ShootingConstants.kShooterReferenceDistanceMeters = SmartDashboard.getNumber("Constants/Shooting/ReferenceDistanceMeters", ShootingConstants.kShooterReferenceDistanceMeters);
    ShootingConstants.kShooterPercentPerMeter = SmartDashboard.getNumber("Constants/Shooting/PercentPerMeter", ShootingConstants.kShooterPercentPerMeter);
    Shooting.updateRateLimiter();
    
    // Auto
    AutoConstants.kMaxSpeedMetersPerSecond = SmartDashboard.getNumber("Constants/Auto/MaxSpeedMetersPerSecond", AutoConstants.kMaxSpeedMetersPerSecond);
    AutoConstants.kMaxAccelerationMetersPerSecondSquared = SmartDashboard.getNumber("Constants/Auto/MaxAccelerationMetersPerSecondSquared", AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    AutoConstants.kMaxAngularSpeedRadiansPerSecond = SmartDashboard.getNumber("Constants/Auto/MaxAngularSpeedRadiansPerSecond", AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared = SmartDashboard.getNumber("Constants/Auto/MaxAngularSpeedRadiansPerSecondSquared", AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
  }

  // Constants
  public static final double MAX_SPEED = Units.feetToMeters(4.5);

  public static final class DrivebaseConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSupportControllerPort = 1;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }


  public static final class IntakeConstants {
    // We will have to change these later
    public static final int kIntakeSparkMaxPort = 10;
    public static double kPercentOutputIntake = 0.50;

    public static double kIntakeAccel = 4.0;
  }

  public static final class IntakeDropConstants {
    // We will have to change these later
    public static final int kIntakeDropSparkMaxPort = 13;
    public static final double kPercentOutputIntakeDrop = 0.70;

  }
  public static final class ShootingConstants {
    // We will need to change these later
    public static final int kSortingSparkMaxPort = 12;
    public static final int kPassthroughSparkMaxPort = 11;
    public static final int kShooterSparkMaxPort = 9;

    public static double kShooterAccel = 4.0;
    
    // We will need to change these later
    public static double kPercentOutputSorting = -0.20;
    public static double kPercentOutputPassthrough = -0.45;
    public static double kPercentOutputShooter = -0.65;
    // Distance (meters) where shooter uses exactly kPercentOutputShooter.
    public static double kShooterReferenceDistanceMeters = 2.0;
    // Additional shooter output applied per meter away from reference distance.
    public static double kShooterPercentPerMeter = 0.25;
  }

  public static final class AutoConstants {
    public static double kMaxSpeedMetersPerSecond = 3;
    public static double kMaxAccelerationMetersPerSecondSquared = 3;
    public static double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}

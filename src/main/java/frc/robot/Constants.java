// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
    public static final double MAX_SPEED = Units.feetToMeters(4.5);
  

  public static final class DrivebaseConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class LiftConstants {
    public static final int kLeftLiftSparkMaxPort = 18;
    public static final int kRightLiftSparkMaxPort = 16;
    public static final double kVoltageLift = 11.5;
  }

  public static final class IntakeConstants {
    // We will have to change these later
    public static final int kIntakeSparkMaxPort = 10;
    public static final double kPercentOutputIntake = 0.90;
  }

  public static final class IntakeDropConstants {
    // We will have to change these later
    public static final int kIntakeDropSparkMaxPort = 13;
    public static final double kPercentOutputIntakeDrop = 0.30;
    public static final int kBottomIntakeDropLimitSwitchPort = 31;
  }
  public static final class ShootingConstants {
    // We will need to change these later
    public static final int kSortingSparkMaxPort = 12;
    public static final int kPassthroughSparkMaxPort = 11;
    public static final int kShooterSparkMaxPort = 9;
    
    // We will need to change these later
    public static final double kPercentOutputSorting = -0.10;
    public static final double kPercentOutputPassthrough = -0.15;
    public static final double kPercentOutputShooter = -0.30;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}

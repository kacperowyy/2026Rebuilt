package frc.robot.subsystems.Vision;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;


public class Position extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    private boolean hasResetPose = false;

    public void resetHasResetPose() {
        hasResetPose = false;
    }

    public Position(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

public void updateOdometryWithVision() {
    swerveDrive.updateOdometry();

    double yaw = swerveDrive.getPose().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation("limelight", yaw, 0, 0, 0, 0, 0);
    SmartDashboard.putNumber("Vision/Gyro Yaw", yaw);

    LimelightHelpers.PoseEstimate measurement;

    if (DriverStation.isDisabled()) {
        // Gdy disabled — używaj MT1 do kalibracji pozycji
        measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    } else {
        // Gdy enabled — używaj MT2
        measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }

    if (measurement == null || measurement.tagCount == 0) {
        SmartDashboard.putString("Vision/Reject Reason", "No tags");
        return;
    }

    if (!measurement.isMegaTag2) {
        // MT1 — wymaga minimum 2 tagów
        if (measurement.tagCount < 2) {
            SmartDashboard.putString("Vision/Reject Reason", "MT1 needs 2 tags");
            return;
        }
        // Reset pozycji gdy disabled
        if (DriverStation.isDisabled()) {
            swerveDrive.resetOdometry(measurement.pose);
            SmartDashboard.putString("Vision/Reject Reason", "None - MT1 reset");
            return;
        }
    } else {
        // MT2 — odrzuć gdy za szybko się obraca
        if (Math.abs(swerveDrive.getRobotVelocity().omegaRadiansPerSecond) > 360 * (Math.PI / 180)) {
            SmartDashboard.putString("Vision/Reject Reason", "Rotating too fast");
            return;
        }
    }

    SmartDashboard.putNumber("Vision/Pose X", measurement.pose.getX());
    SmartDashboard.putNumber("Vision/Pose Y", measurement.pose.getY());
    SmartDashboard.putNumber("Vision/Pose Rotation", measurement.pose.getRotation().getDegrees());

    swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
    swerveDrive.addVisionMeasurement(measurement.pose, measurement.timestampSeconds);

    SmartDashboard.putString("Vision/Reject Reason", "None");
    SmartDashboard.putBoolean("Vision/Measurement Applied", true);
}

}


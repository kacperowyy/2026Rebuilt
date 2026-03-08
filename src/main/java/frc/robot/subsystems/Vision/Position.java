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

    double rawYaw = swerveDrive.getPose().getRotation().getDegrees();

    // Dla MT2 dodaj 180° dla czerwonej drużyny
    double yawForMT2 = rawYaw;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        yawForMT2 = rawYaw + 180;
    }
    LimelightHelpers.SetRobotOrientation("limelight", yawForMT2, 0, 0, 0, 0, 0);

    // Log current yaw
    SmartDashboard.putNumber("Vision/Gyro Yaw", yawForMT2);

    // Read MegaTag2
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        SmartDashboard.putBoolean("Vision/FirstTag", hasResetPose);
    // Log Limelight data
    boolean hasValidTags = mt2 != null && mt2.tagCount > 0;
    SmartDashboard.putBoolean("Vision/Has Valid Tags", hasValidTags);
    SmartDashboard.putNumber("Vision/Tag Count", mt2 != null ? mt2.tagCount : 0);

    // Reject invalid measurements
    if (Math.abs(yawForMT2) > 720) {
        DriverStation.reportWarning("[Vision] Measurement rejected – yaw out of range: " + yawForMT2, false);
        SmartDashboard.putString("Vision/Reject Reason", "Yaw > 720°");
        SmartDashboard.putBoolean("Vision/Measurement Applied", false);
        return;
    }

    if (mt2 == null || mt2.tagCount == 0) {
        SmartDashboard.putString("Vision/Reject Reason", "No tags");
        SmartDashboard.putBoolean("Vision/Measurement Applied", false);
        return;
    }

        if (Math.abs(swerveDrive.getRobotVelocity().omegaRadiansPerSecond) > 360 * (Math.PI / 180)) {
            SmartDashboard.putString("Vision/Reject Reason", "Rotating too fast");
            SmartDashboard.putBoolean("Vision/Measurement Applied", false);
            return;
        }

    // Log pose before applying
    Pose2d estimatedPose = mt2.pose;
    SmartDashboard.putNumber("Vision/Pose X",        estimatedPose.getX());
    SmartDashboard.putNumber("Vision/Pose Y",        estimatedPose.getY());
    SmartDashboard.putNumber("Vision/Pose Rotation", estimatedPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Vision/Timestamp",     mt2.timestampSeconds);
    SmartDashboard.putNumber("Vision/Avg Tag Dist",  mt2.avgTagDist);
    
    if (!hasResetPose) {
        LimelightHelpers.PoseEstimate mt1 = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (mt1 != null && mt1.tagCount > 0
                && mt1.rawFiducials[0].ambiguity < 0.7) {

            Pose2d pose = FlippingUtil.flipFieldPose(mt1.pose);

            SmartDashboard.putNumber("Vision/First Pose X",        pose.getX());
            SmartDashboard.putNumber("Vision/First Pose Y",        pose.getY());
            SmartDashboard.putNumber("Vision/First Pose Rotation", pose.getRotation().getDegrees());

            swerveDrive.setGyro(new Rotation3d(0, 0, pose.getRotation().getRadians()));


            hasResetPose = true;
        }
        return;
    }

    // Apply vision measurement
    swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
    swerveDrive.addVisionMeasurement(estimatedPose, mt2.timestampSeconds);

    SmartDashboard.putString("Vision/Reject Reason",      "None");
    SmartDashboard.putBoolean("Vision/Measurement Applied", true);
    SmartDashboard.putNumber("Vision/Updates This Match",
        SmartDashboard.getNumber("Vision/Updates This Match", 0) + 1);
}

}


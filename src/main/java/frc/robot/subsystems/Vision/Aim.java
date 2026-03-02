package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Aim extends SubsystemBase {

    public Aim() {
        
    }

    public Pose2d findPoseForShoot() {

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        Pose2d robotPose = mt2.pose; 
        Pose3d fiducialPose = mt2.rawFiducials[0].pose;

        double shootingDistance = 2.5; //distance for tower
        Translation2d tagTranslation2d = 
            new Translation2d(fiducialPose.getX(), fiducialPose.getY());

        Translation2d direction = tagTranslation2d.div(tagTranslation2d.getNorm());

        Translation2d targetTranslation =
            robotPose.getTranslation().plus(direction.times(tagTranslation2d.getNorm() - shootingDistance));

        Rotation2d targetRotation = new Rotation2d(
            tagTranslation2d.getX(),
            tagTranslation2d.getY()
        );

        Pose2d shootingPose = new Pose2d(targetTranslation, targetRotation);

        return shootingPose;
    }
}

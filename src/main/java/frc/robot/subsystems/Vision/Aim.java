package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.MyFieldLayouts;

import java.util.Optional;
import java.util.Set;

public class Aim extends SubsystemBase {

    private final AprilTagFieldLayout fieldLayout;

    private static final Set<Integer> BLUE_REEF_TAGS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
    private static final Set<Integer> RED_REEF_TAGS  = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

    public Aim() {
        fieldLayout = MyFieldLayouts.createRebuilt2026Layout();
    }

    private Set<Integer> getTargetTags() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            System.out.println("[AIM] Alliance not set! Defaulting to Blue.");
            return BLUE_REEF_TAGS;
        }
        return alliance.get() == DriverStation.Alliance.Red ? RED_REEF_TAGS : BLUE_REEF_TAGS;
    }

    public Pose2d findPoseForShoot() {

        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (mt2 == null) {
            System.out.println("[AIM] mt2 is NULL");
            return null;
        }

        if (mt2.rawFiducials == null || mt2.rawFiducials.length == 0) {
            System.out.println("[AIM] No AprilTags detected");
            return null;
        }

        Set<Integer> validTags = getTargetTags();

        LimelightHelpers.RawFiducial targetFiducial = null;
        for (LimelightHelpers.RawFiducial fiducial : mt2.rawFiducials) {
            if (validTags.contains(fiducial.id)) {
                targetFiducial = fiducial;
                break;
            }
        }

        if (targetFiducial == null) {
            System.out.println("[AIM] No reef tag detected. Visible tags:");
            for (LimelightHelpers.RawFiducial f : mt2.rawFiducials) {
                System.out.println("  - Tag ID: " + f.id);
            }
            return null;
        }

        Pose2d robotPose = mt2.pose;
        int tagID = targetFiducial.id;

        System.out.println("[AIM] Using Reef Tag ID: " + tagID);
        System.out.println("[AIM] Robot Pose X: " + robotPose.getX() + " Y: " + robotPose.getY());

        Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(tagID);

        if (tagPoseOptional.isEmpty()) {
            System.out.println("[AIM] Tag ID " + tagID + " not found in fieldLayout");
            return null;
        }

        Pose3d fiducialPose = tagPoseOptional.get();

        System.out.println("[AIM] Tag Pose X: " + fiducialPose.getX() + " Y: " + fiducialPose.getY());

        Translation2d tagTranslation = new Translation2d(fiducialPose.getX(), fiducialPose.getY());

        // Vector from robot to tag
        Translation2d toTag = tagTranslation.minus(robotPose.getTranslation());
        double distToTag = toTag.getNorm();

        System.out.println("[AIM] Distance to tag: " + distToTag);

        double shootingDistance = 1.5; // desired distance from tag in meters

        // Unit vector pointing from tag back toward the robot (away from tower)
        Translation2d directionToTag = toTag.div(distToTag);

        // Target position is exactly shootingDistance meters away from the tag,
        // along the robot->tag line. Works whether the robot needs to drive forward OR back up.
        Translation2d targetTranslation = tagTranslation.minus(directionToTag.times(shootingDistance));

        // Rotation facing toward the tag
        Rotation2d targetRotation = new Rotation2d(toTag.getX(), toTag.getY());
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetRotation = targetRotation.plus(Rotation2d.fromDegrees(180));
        }

        Pose2d shootingPose = new Pose2d(targetTranslation, targetRotation);

        System.out.println("[AIM] Distance to tag: " + distToTag
                         + (distToTag < shootingDistance ? " (too close — backing up)" : " (approaching)"));
        System.out.println("[AIM] Shooting Pose X: " + shootingPose.getX()
                         + " Y: " + shootingPose.getY()
                         + " Rot: " + shootingPose.getRotation().getDegrees());

        SmartDashboard.putNumber("Shoot position X", shootingPose.getX());
        SmartDashboard.putNumber("Shoot position Y", shootingPose.getY());
        SmartDashboard.putNumber("Shoot position Rot", shootingPose.getRotation().getDegrees());

        return shootingPose;
    }
}
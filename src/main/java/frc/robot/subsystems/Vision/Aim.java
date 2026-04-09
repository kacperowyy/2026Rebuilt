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

    private static final Set<Integer> BLUE_CENTER_TAGS = Set.of(24, 26, 27);
    private static final Set<Integer> RED_CENTER_TAGS  = Set.of(8, 10, 11);

    private static final Set<Integer> BLUE_VALID_TAGS  = Set.of(18, 21, 24, 25, 26, 27);
    private static final Set<Integer> RED_VALID_TAGS   = Set.of(2, 5, 8, 9, 10, 11);

    public Aim() {
        fieldLayout = MyFieldLayouts.createRebuilt2026Layout();
    }

    private Set<Integer> getTargetTags() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return BLUE_VALID_TAGS;
        }
        return alliance.get() == DriverStation.Alliance.Red ? RED_VALID_TAGS : BLUE_VALID_TAGS;
    }

    private Set<Integer> getCenterTags() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_CENTER_TAGS;
        }
        return BLUE_CENTER_TAGS;
    }

    public Pose2d findPoseForShoot() {

        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (mt2 == null) {
            return null;
        }

        if (mt2.rawFiducials == null || mt2.rawFiducials.length == 0) {
            return null;
        }

        Set<Integer> validTags = getTargetTags();
        Set<Integer> centerTags = getCenterTags();

        LimelightHelpers.RawFiducial targetFiducial = null;
        
        // Priority 1: Center Tag
        for (LimelightHelpers.RawFiducial fiducial : mt2.rawFiducials) {
            if (centerTags.contains(fiducial.id)) {
                targetFiducial = fiducial;
                break;
            }
        }

        // Priority 2: Any valid tag if center is missing
        if (targetFiducial == null) {
            for (LimelightHelpers.RawFiducial fiducial : mt2.rawFiducials) {
                if (validTags.contains(fiducial.id)) {
                    targetFiducial = fiducial;
                    break;
                }
            }
        }

        if (targetFiducial == null) {
            return null;
        }

        Pose2d robotPose = mt2.pose;
        int tagID = targetFiducial.id;

        Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(tagID);

        if (tagPoseOptional.isEmpty()) {
            return null;
        }

        Pose3d fiducialPose = tagPoseOptional.get();

        Translation2d tagTranslation = new Translation2d(fiducialPose.getX(), fiducialPose.getY());

        // Vector from robot to tag
        Translation2d toTag = tagTranslation.minus(robotPose.getTranslation());
        double distToTag = toTag.getNorm();

        double shootingDistance = 1.5; // desired distance from tag in meters

        // Unit vector pointing from tag back toward the robot (away from tower)
        Translation2d directionToTag = toTag.div(distToTag);

        // Target position is exactly shootingDistance meters away from the tag,
        // along the robot->tag line. Works whether the robot needs to drive forward OR back up.
        Translation2d targetTranslation = tagTranslation.minus(directionToTag.times(shootingDistance));

        // Rotation facing toward the tag
        Rotation2d targetRotation = new Rotation2d(toTag.getX(), toTag.getY());
        
        Pose2d shootingPose = new Pose2d(targetTranslation, targetRotation);

        SmartDashboard.putNumber("Shoot position X", shootingPose.getX());
        SmartDashboard.putNumber("Shoot position Y", shootingPose.getY());
        SmartDashboard.putNumber("Shoot position Rot", shootingPose.getRotation().getDegrees());

        return shootingPose;
    }
}
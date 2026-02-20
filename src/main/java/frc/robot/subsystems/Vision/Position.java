package frc.robot.subsystems.Vision;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;


public class Position extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    public static boolean doRejectUpdate = false;
    public static boolean twoTags = false;

    private final Pigeon2 m_gyro = new Pigeon2(0);

    public Position(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public void updateOdometryWithVision() {
        // aktualizacja odometrii
        swerveDrive.updateOdometry();

        // ustawienie headingu w Limelight
        LimelightHelpers.SetRobotOrientation(
            "limelight",
            m_gyro.getYaw().getValueAsDouble(), 
            0, 0, 0, 0, 0
        );
        

        // odczyt MegaTag2
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        // odrzucenie błędnych pomiarów
        if (Math.abs(m_gyro.getYaw().getValueAsDouble()) > 720 || mt2.tagCount == 0) return;

        // dodanie danych vision
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }

    public void updateRobotPose2D() {
        doRejectUpdate = false;
        twoTags = false;
        String rejectReason = "None"; 

        
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > 0.7) {
                doRejectUpdate = true;
                rejectReason = "High Ambiguity"; 
            }
            if (mt1.rawFiducials[0].distToCamera > 4) {
                doRejectUpdate = true;
                rejectReason = "Too Far";
            }
        }
        if (mt1.tagCount == 0) {
            doRejectUpdate = true;
            rejectReason = "No Tags";
        }

        twoTags = mt1.tagCount == 2 ? true : false;

        SmartDashboard.putBoolean("Vision Update Rejected", doRejectUpdate);
        SmartDashboard.putString("Vision Reject Reason", rejectReason);

        if (!doRejectUpdate) {
            Pose2d robotPose = mt1.pose;
            double timestamp = mt1.timestampSeconds;

            // Update'ujemy pozycje tylko wtedy, kiedy dane sa ok
            swerveDrive.addVisionMeasurement(robotPose, timestamp);
        }
    }

    @Override
    public void periodic() {
        updateOdometryWithVision();
    }
}

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShootingConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;

public class Shooting extends SubsystemBase {
    private static final String LIMELIGHT_NAME = "limelight";
    private static final Set<Integer> BLUE_CENTER_TAGS = Set.of(24, 26, 27);
    private static final Set<Integer> RED_CENTER_TAGS  = Set.of(8, 10, 11);

    private static final Set<Integer> BLUE_VALID_TAGS  = Set.of(18, 21, 24, 25, 26, 27);
    private static final Set<Integer> RED_VALID_TAGS   = Set.of(2, 5, 8, 9, 10, 11);

    private final SparkMax sortingMotor = new SparkMax(ShootingConstants.kSortingSparkMaxPort, MotorType.kBrushless);
    private final SparkMax passthroughMotor = new SparkMax(ShootingConstants.kPassthroughSparkMaxPort, MotorType.kBrushless);
    private final SparkMax shooterMotor = new SparkMax(ShootingConstants.kShooterSparkMaxPort, MotorType.kBrushless);

    private final SparkMaxConfig sortingMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig passthroughMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();

    private static double lastShooterAccel = ShootingConstants.kShooterAccel;
    private static SlewRateLimiter rateLimiter = new SlewRateLimiter(ShootingConstants.kShooterAccel);

    private double shooterTargetSpeed = 0.0;
    private boolean shootingActive = false;
    private boolean feedersEnabled = false;

    public Shooting() {
        sortingMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
        passthroughMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
        shooterMotorConfig.inverted(false).idleMode(IdleMode.kBrake);

        sortingMotor.configure(sortingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        passthroughMotor.configure(passthroughMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void startShooting() {
        if (!shootingActive) {
            shootingActive = true;
            feedersEnabled = false;
        }

        shooterTargetSpeed = getShooterSpeedFromLiveTagDistance();
    }

    @Override
    public void periodic() {
        double shooterOutput = rateLimiter.calculate(shooterTargetSpeed);
        shooterMotor.set(shooterOutput);
        SmartDashboard.putNumber("Shooter/AppliedOutput", shooterOutput);

        if (!shootingActive || shooterTargetSpeed == 0.0) {
            feedersEnabled = false;
        } else if (!feedersEnabled && hasReachedShooterTarget(shooterOutput)) {
            feedersEnabled = true;
        }

        setFeedersEnabled(feedersEnabled);
    }

    public void sortAndPass() {
        sortingMotor.set(ShootingConstants.kPercentOutputSorting);
        passthroughMotor.set(ShootingConstants.kPercentOutputPassthrough);
    }

    public void sortAndPassReverse() {
        sortingMotor.set(-ShootingConstants.kPercentOutputSorting);
        passthroughMotor.set(-ShootingConstants.kPercentOutputPassthrough);
    }

    public void stop() {
        shootingActive = false;
        feedersEnabled = false;
        setFeedersEnabled(false);
        shooterTargetSpeed = 0.0;
    }

    private void setFeedersEnabled(boolean enabled) {
        sortingMotor.set(enabled ? ShootingConstants.kPercentOutputSorting : 0.0);
        passthroughMotor.set(enabled ? ShootingConstants.kPercentOutputPassthrough : 0.0);
    }

    private boolean hasReachedShooterTarget(double shooterOutput) {
        if (shooterTargetSpeed < 0.0) {
            return shooterOutput <= shooterTargetSpeed;
        }
        return shooterOutput >= shooterTargetSpeed;
    }

    private Set<Integer> getTargetTags() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_VALID_TAGS;
        }
        return BLUE_VALID_TAGS;
    }

    private Set<Integer> getCenterTags() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_CENTER_TAGS;
        }
        return BLUE_CENTER_TAGS;
    }

    private double getShooterSpeedFromLiveTagDistance() {
        double baseShooterOutput = clampShooterOutput(ShootingConstants.kPercentOutputShooter);
        double distanceMeters = getClosestTargetTagDistanceMeters();

        if (!Double.isFinite(distanceMeters)) {
            SmartDashboard.putBoolean("Shooter/TagDistanceValid", false);
            SmartDashboard.putNumber("Shooter/TargetOutput", baseShooterOutput);
            return baseShooterOutput;
        }

        double distanceFromReferenceMeters = distanceMeters - ShootingConstants.kShooterReferenceDistanceMeters;
        double distanceAdjustment = distanceFromReferenceMeters * ShootingConstants.kShooterPercentPerMeter;
        double shooterOutput = clampShooterOutput(baseShooterOutput - distanceAdjustment);

        SmartDashboard.putBoolean("Shooter/TagDistanceValid", true);
        SmartDashboard.putNumber("Shooter/TagDistanceM", distanceMeters);
        SmartDashboard.putNumber("Shooter/ReferenceDistanceM", ShootingConstants.kShooterReferenceDistanceMeters);
        SmartDashboard.putNumber("Shooter/TargetOutput", shooterOutput);

        return shooterOutput;
    }

    private double getClosestTargetTagDistanceMeters() {
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        if (mt2 == null || mt2.rawFiducials == null || mt2.rawFiducials.length == 0) {
            return Double.NaN;
        }

        double closestDistance = Double.POSITIVE_INFINITY;
        Set<Integer> targetTags = getTargetTags();
        Set<Integer> centerTags = getCenterTags();

        // Priority 1: Distance to the exact center tag
        for (LimelightHelpers.RawFiducial fiducial : mt2.rawFiducials) {
            if (centerTags.contains(fiducial.id)) {
                double measuredDistance = fiducial.distToRobot > 0.0 ? fiducial.distToRobot : fiducial.distToCamera;
                if (measuredDistance > 0.0) {
                    if (measuredDistance < closestDistance) {
                        closestDistance = measuredDistance;
                    }
                }
            }
        }
        
        if (Double.isFinite(closestDistance)) {
            return closestDistance;
        }

        // Priority 2: Closest of the other valid tags
        for (LimelightHelpers.RawFiducial fiducial : mt2.rawFiducials) {
            if (!targetTags.contains(fiducial.id)) {
                continue;
            }

            double measuredDistance = fiducial.distToRobot > 0.0 ? fiducial.distToRobot : fiducial.distToCamera;
            if (measuredDistance > 0.0 && measuredDistance < closestDistance) {
                closestDistance = measuredDistance;
            }
        }

        return Double.isFinite(closestDistance) ? closestDistance : Double.NaN;
    }

    private double clampShooterOutput(double requestedOutput) {
        return MathUtil.clamp(requestedOutput, -1.0, 1.0);
    }

    static public void updateRateLimiter() {
        if (lastShooterAccel == ShootingConstants.kShooterAccel) {
            return;
        }
        lastShooterAccel = ShootingConstants.kShooterAccel;
        rateLimiter = new SlewRateLimiter(ShootingConstants.kShooterAccel);
    }
}

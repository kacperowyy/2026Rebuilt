package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ShootingConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooting extends SubsystemBase {
    // Define motors
    private final SparkMax sortingMotor = new SparkMax(ShootingConstants.kSortingSparkMaxPort, MotorType.kBrushless);
    private final SparkMax passthroughMotor = new SparkMax(ShootingConstants.kPassthroughSparkMaxPort, MotorType.kBrushless);
    private final SparkMax shooterMotor = new SparkMax(ShootingConstants.kShooterSparkMaxPort, MotorType.kBrushless);

    // Motor configurations
    private final SparkMaxConfig sortingMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig passthroughMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
    public Shooting() {
        sortingMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
        passthroughMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
        shooterMotorConfig.inverted(false).idleMode(IdleMode.kBrake);

        sortingMotor.configure(sortingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        passthroughMotor.configure(passthroughMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Methods to control motors
    public void start() {
        sortingMotor.set(ShootingConstants.kPercentOutputSorting);
        passthroughMotor.set(ShootingConstants.kPercentOutputPassthrough);
        shooterMotor.set(ShootingConstants.kPercentOutputShooter);
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
        sortingMotor.set(0);
        passthroughMotor.set(0);
        shooterMotor.set(0);
    }
}
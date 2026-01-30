package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.LiftConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
    private final SparkMax liftLeftMotor = new SparkMax(LiftConstants.kLiftSparkMaxPort, MotorType.kBrushed);
    private final SparkMaxConfig liftMotorConfig = new SparkMaxConfig();

    public Lift() {
        liftMotorConfig.inverted(false).idleMode(IdleMode.kBrake);

        liftLeftMotor.configure(liftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runUp() {
        liftLeftMotor.setVoltage(11);
    }

    public void runDown() {
        liftLeftMotor.setVoltage(-11);
    }

    public void stop() {
        liftLeftMotor.setVoltage(0);
    }
}

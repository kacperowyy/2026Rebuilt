package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // Define motor
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.kIntakeSparkMaxPort, MotorType.kBrushless);

    // Motor configurations
    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

    public Intake() {
        intakeMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void start() {
        intakeMotor.set(IntakeConstants.kPercentOutputIntake);
    }

    public void stop() {
        intakeMotor.set(0);
    }
}

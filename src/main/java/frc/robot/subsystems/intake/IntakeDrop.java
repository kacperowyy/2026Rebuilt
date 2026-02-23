package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeDropConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeDrop extends SubsystemBase {
    // Define motor
    private final SparkMax intakeMotor = new SparkMax(IntakeDropConstants.kIntakeDropSparkMaxPort, MotorType.kBrushless);

    // Motor configurations
    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();



    public IntakeDrop() {
        intakeMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }



    public void start() {
        intakeMotor.set(IntakeDropConstants.kPercentOutputIntakeDrop); 
    }

    public void stop() {
        intakeMotor.set(0);
    }
}
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
        // Slow down the motor slowly & Log the value of shooter motor
        System.out.println("Intake motor value: " + intakeMotor.get());

        Runnable speedUp = () -> {
            while (intakeMotor.get() < IntakeConstants.kPercentOutputIntake) {
                intakeMotor.set(intakeMotor.get() + 0.05);
                // Wait
                try {
                    Thread.sleep(50); // Sleep for 50 milliseconds
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt(); // Restore interrupted status
                }
            }
            intakeMotor.set(IntakeConstants.kPercentOutputIntake);
        };

        // Slow down the motor in a separate thread to avoid blocking the main thread
        Thread speedUpThread = new Thread(speedUp);
        speedUpThread.start();
    }

    public void reverse() {
        intakeMotor.set(-IntakeConstants.kPercentOutputIntake);
    }

    public void stop() {
        // We will later make this method slow down the motor slowly from both positive and negative values
        intakeMotor.set(0);
    }
}

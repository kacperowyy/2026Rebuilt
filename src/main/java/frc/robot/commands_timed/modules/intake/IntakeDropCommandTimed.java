package frc.robot.commands_timed.modules.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.IntakeDrop;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDropCommandTimed extends Command {
    private final IntakeDrop intakeDrop;
    private final Timer timer = new Timer();
    private final double duration;  

    public IntakeDropCommandTimed(IntakeDrop intakeDrop, double durationSeconds) {
        this.intakeDrop = intakeDrop;
        this.duration = durationSeconds;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        intakeDrop.start();   
    }

    @Override
    public void end(boolean interrupted) {
        intakeDrop.stop();
        timer.stop();
    } 

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

}
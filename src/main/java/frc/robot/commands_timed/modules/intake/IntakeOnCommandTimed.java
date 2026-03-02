package frc.robot.commands_timed.modules.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeOnCommandTimed extends Command {
    private final Intake intake;
    private final Timer timer = new Timer();
    private final double duration;

    public IntakeOnCommandTimed(Intake intake, double durationSeconds) {
        this.intake = intake;
        this.duration = durationSeconds;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        intake.start();   
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        timer.stop();
    } 

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

}
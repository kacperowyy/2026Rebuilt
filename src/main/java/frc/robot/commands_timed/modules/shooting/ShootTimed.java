package frc.robot.commands_timed.modules.shooting;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Shooting;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootTimed extends Command {
    private final Shooting shooting;
    private final Timer timer = new Timer();
    private final double duration;

    public ShootTimed(Shooting shooting, double durationSeconds) {
        this.shooting = shooting;
        this.duration = durationSeconds;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        shooting.startShooting();   
    }

    @Override
    public void end(boolean interrupted) {
        shooting.stop();
        timer.stop();
    } 

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

}
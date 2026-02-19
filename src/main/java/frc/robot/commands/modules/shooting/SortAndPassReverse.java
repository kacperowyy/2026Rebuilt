package frc.robot.commands.modules.shooting;

import frc.robot.subsystems.Shooting;

import edu.wpi.first.wpilibj2.command.Command;

public class SortAndPassReverse extends Command {
    private final Shooting shooting;

    public SortAndPassReverse(Shooting shooting) {
        this.shooting = shooting;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooting.sortAndPassReverse();   
    }

    @Override
    public void end(boolean interrupted) {
        shooting.stop();
    } 

    @Override
    public boolean isFinished() {
        return false;
    }

}
package frc.robot.commands.moduls.lift;

import frc.robot.subsystems.Lift;

import edu.wpi.first.wpilibj2.command.Command;

public class RaiseLiftCommand extends Command {
    private final Lift lift;

    public RaiseLiftCommand(Lift lift) {
        this.lift = lift;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
         lift.runUp();   
    }

    @Override
    public void end(boolean interrupted) {
        lift.stop();
    } 

    @Override
    public boolean isFinished() {
        return false;
    }


}

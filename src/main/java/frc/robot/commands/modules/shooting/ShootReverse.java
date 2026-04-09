package frc.robot.commands.modules.shooting;

import frc.robot.subsystems.Shooting;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootReverse extends Command {
    private final Shooting shooting;

    public ShootReverse(Shooting shooting) {
        this.shooting = shooting;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooting.reverseShoot();
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
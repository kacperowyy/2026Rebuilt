package frc.robot.commands.modules.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooting;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AimAndShoot extends Command {
    private final SwerveSubsystem drivebase;
    private final Shooting shooting;
    private boolean isShooting;

    public AimAndShoot(SwerveSubsystem drivebase, Shooting shooting) {
        this.drivebase = drivebase;
        this.shooting = shooting;
        addRequirements(drivebase, shooting);
    }

    @Override
    public void initialize() {
        drivebase.cancelAimAndDrive();
        drivebase.saveTowerHeading();
        isShooting = false;
    }

    @Override
    public void execute() {
        drivebase.faceSavedTarget();
        if (!isShooting && drivebase.isFacingSavedTarget()) {
            isShooting = true;
        }
        if (isShooting) {
            shooting.startShooting();
        }
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

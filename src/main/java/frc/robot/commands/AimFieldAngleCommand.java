package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class AimFieldAngleCommand extends Command {

    private final TurretSubsystem turret;
    private final double fieldAngleDeg;

    public AimFieldAngleCommand(TurretSubsystem turret, double fieldAngleDeg) {
        this.turret = turret;
        this.fieldAngleDeg = fieldAngleDeg;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.aimFieldRelative(fieldAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

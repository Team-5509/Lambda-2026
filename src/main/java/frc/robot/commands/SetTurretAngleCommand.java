package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class SetTurretAngleCommand extends Command {

    private final TurretSubsystem turret;
    private final double angleDeg;

    public SetTurretAngleCommand(TurretSubsystem turret, double angleDeg) {
        this.turret = turret;
        this.angleDeg = angleDeg;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setTurretAngleDegrees(angleDeg);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

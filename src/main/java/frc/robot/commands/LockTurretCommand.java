package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Locks the turret at a fixed angle (degrees) read from SmartDashboard.
 * The angle can be changed live via "TurretLock/AngleDegrees" on the dashboard.
 * The turret motor remains in Brake mode, so it holds position even under load.
 * This command runs until interrupted (e.g. when "TurretLock/Enabled" is set false).
 */
public class LockTurretCommand extends Command {

    private final TurretSubsystem turret;

    public LockTurretCommand(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double angleDegrees = SmartDashboard.getNumber("TurretLock/AngleDegrees", 0.0);
        turret.setTurretAngleDegrees(angleDegrees);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}

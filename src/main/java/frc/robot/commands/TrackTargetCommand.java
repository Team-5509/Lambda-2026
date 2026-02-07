package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TrackTargetCommand extends Command {

    private final TurretSubsystem turret;
    private final DoubleSupplier targetFieldAngleSupplier;

    public TrackTargetCommand(
            TurretSubsystem turret,
            DoubleSupplier targetFieldAngleSupplier
    ) {
        this.turret = turret;
        this.targetFieldAngleSupplier = targetFieldAngleSupplier;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.aimFieldRelative(targetFieldAngleSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

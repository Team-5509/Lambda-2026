package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TrackFieldPoseCommand extends Command {

    private final TurretSubsystem turret;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Translation2d> robotVelocitySupplier;
    private final Translation2d targetPosition;

    private final double projectileSpeed;

    /**
     * @param turret                Turret subsystem
     * @param robotPoseSupplier     Robot pose (field-relative)
     * @param robotVelocitySupplier Robot velocity (field-relative, m/s)
     * @param targetPosition        Fixed field target position
     * @param projectileSpeed       Projectile speed (m/s)
     */
    public TrackFieldPoseCommand(
            TurretSubsystem turret,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Translation2d> robotVelocitySupplier,
            Translation2d targetPosition,
            double projectileSpeed) {
        this.turret = turret;
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.targetPosition = targetPosition;
        this.projectileSpeed = projectileSpeed;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.aimFieldRelativeWithPrediction(
                robotPoseSupplier.get(),
                robotVelocitySupplier.get(),
                targetPosition,
                projectileSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

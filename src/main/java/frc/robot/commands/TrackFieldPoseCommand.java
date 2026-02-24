package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class TrackFieldPoseCommand extends Command {

    private final TurretSubsystem turret;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Translation2d> robotVelocitySupplier;

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
            // Translation2d targetPosition,
            double projectileSpeed) {
        this.turret = turret;
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        // this.targetPosition = targetPosition;
        this.projectileSpeed = projectileSpeed;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        Translation2d targetPosition = Constants.TurretSubsystemConstants.blueHubPose; // default target
        Boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        if (isBlue && robotPoseSupplier.get().getTranslation().getX() < Constants.TurretSubsystemConstants.blueLineZone) {
            targetPosition = Constants.TurretSubsystemConstants.blueHubPose;
        } else if (!isBlue && robotPoseSupplier.get().getTranslation().getX() > Constants.TurretSubsystemConstants.redLineZone) {
            targetPosition = Constants.TurretSubsystemConstants.redHubPose;
        } else {
            targetPosition = isBlue ? Constants.TurretSubsystemConstants.blueHomePose : Constants.TurretSubsystemConstants.redHomePose; // default to blue home if isBlue is true, otherwise red home
        }
        SmartDashboard.putString("TrackFieldPoseCommand/Target", isBlue ? "Blue Hub/Home" : "Red Hub/Home" + targetPosition.toString());
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

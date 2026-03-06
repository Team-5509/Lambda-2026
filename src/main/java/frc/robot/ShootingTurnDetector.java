package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Constants.TurretSubsystemConstants;

/**
 * Detects when it is our robot's turn to shoot during a match.
 *
 * <p>A "shooting turn" is active when ALL of the following are true:
 * <ol>
 *   <li>The robot is enabled in teleop or autonomous.</li>
 *   <li>The robot is positioned on its own side of the field (the side
 *       where it shoots at the hub), as defined by the field zone constants.</li>
 * </ol>
 *
 * <p>Usage in RobotContainer:
 * <pre>
 *   ShootingTurnDetector detector = new ShootingTurnDetector(() -> drivetrain.getState().Pose);
 *   detector.shootingTurn().whileTrue(makeLaunchLookup());
 * </pre>
 *
 * <p>Call {@link #update()} every robot loop (e.g. from {@code robotPeriodic()}) so that
 * SmartDashboard values stay current.
 */
public class ShootingTurnDetector {

    private final Supplier<Pose2d> m_poseSupplier;

    /** True when the robot is on its own (hub-shooting) side of the field. */
    private boolean m_inShootingZone = false;

    /** True when the robot is enabled (teleop or auto). */
    private boolean m_robotEnabled = false;

    /**
     * @param poseSupplier Supplier for the current field-relative robot pose.
     *                     Typically {@code () -> drivetrain.getState().Pose}.
     */
    public ShootingTurnDetector(Supplier<Pose2d> poseSupplier) {
        m_poseSupplier = poseSupplier;
    }

    /**
     * Returns a {@link Trigger} that is active whenever it is our shooting turn.
     *
     * <p>Bind commands to this trigger in {@code RobotContainer.configureBindings()}:
     * <pre>
     *   detector.shootingTurn().whileTrue(makeLaunchLookup());
     * </pre>
     */
    public Trigger shootingTurn() {
        return new Trigger(this::isShootingTurn);
    }

    /**
     * Returns {@code true} when all shooting-turn conditions are met:
     * <ul>
     *   <li>Robot is enabled (teleop or auto)</li>
     *   <li>Robot is on its own hub-shooting side of the field</li>
     * </ul>
     */
    public boolean isShootingTurn() {
        return m_robotEnabled && m_inShootingZone;
    }

    /**
     * Must be called every robot loop (e.g. from {@code Robot.robotPeriodic()}).
     * Updates internal state and publishes values to SmartDashboard.
     */
    public void update() {
        m_robotEnabled = DriverStation.isEnabled()
                && (DriverStation.isTeleop() || DriverStation.isAutonomous());

        Pose2d pose = m_poseSupplier.get();
        double robotX = pose.getTranslation().getX();

        boolean isBlue = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

        // On the blue alliance, the hub-shooting zone is x < blueLineZone.
        // On the red alliance, the hub-shooting zone is x > redLineZone.
        if (isBlue) {
            m_inShootingZone = robotX < TurretSubsystemConstants.blueLineZone;
        } else {
            m_inShootingZone = robotX > TurretSubsystemConstants.redLineZone;
        }

        // Publish diagnostics so drivers/operators can see the state on the dashboard.
        SmartDashboard.putBoolean("ShootingTurn/IsOurTurn", isShootingTurn());
        SmartDashboard.putBoolean("ShootingTurn/RobotEnabled", m_robotEnabled);
        SmartDashboard.putBoolean("ShootingTurn/InShootingZone", m_inShootingZone);
        SmartDashboard.putString("ShootingTurn/Alliance", isBlue ? "Blue" : "Red");
    }
}

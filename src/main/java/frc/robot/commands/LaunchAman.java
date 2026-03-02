// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Algorithms.ShootingArc;
import frc.robot.Constants.Constants.LauncherSubsystemConstants;
import frc.robot.Constants.Constants.TurretSubsystemConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Spins up the launcher and feeds the ball.
 *
 * Hood position and launcher speed are derived each loop from:
 *   - Current robot pose (field-relative)
 *   - Current robot velocity (field-relative) — used for lead compensation
 *   - Hub position from Constants
 *   - Projectile-motion physics (fixed ball speed from Constants)
 *
 * Two calibration constants must be measured on the robot before this command
 * produces accurate results:
 *   LauncherSubsystemConstants.kHoodMinAngleDeg / kHoodMaxAngleDeg  (hood angle at rotation limits)
 *   LauncherSubsystemConstants.kLauncherWheelDiameterM              (flywheel diameter)
 */
public class LaunchAman extends Command {
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final LauncherSubsystem m_launcherSubsystem;
  private final KickerSubsystem   m_kickerSubsystem;
  private final Supplier<Pose2d>        m_poseSupplier;
  private final Supplier<Translation2d> m_velocitySupplier; // field-relative, m/s

  private final Timer m_timer = new Timer();
  private final ShootingArc m_shootingArc = new ShootingArc();

  // Updated each execute() call
  private double m_speed   = 50.0;
  private double m_hoodPos = LauncherSubsystemConstants.kHoodMinRot;

  /**
   * @param conveyorSubsystem  Conveyor subsystem
   * @param launcherSubsystem  Launcher subsystem
   * @param kickerSubsystem    Kicker subsystem
   * @param poseSupplier       Field-relative robot pose (e.g. drivetrain::getState().Pose)
   * @param velocitySupplier   Field-relative robot velocity in m/s (e.g. getFieldRelativeVelocity)
   */
  public LaunchAman(
      ConveyorSubsystem conveyorSubsystem,
      LauncherSubsystem launcherSubsystem,
      KickerSubsystem kickerSubsystem,
      Supplier<Pose2d> poseSupplier,
      Supplier<Translation2d> velocitySupplier) {
    m_conveyorSubsystem = conveyorSubsystem;
    m_launcherSubsystem = launcherSubsystem;
    m_kickerSubsystem   = kickerSubsystem;
    m_poseSupplier      = poseSupplier;
    m_velocitySupplier  = velocitySupplier;

    addRequirements(conveyorSubsystem, launcherSubsystem, kickerSubsystem);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    updateShootingParams();

    m_launcherSubsystem.extendHoodMM(m_hoodPos);
    m_launcherSubsystem.runLauncherMM(m_speed);

    if (m_timer.hasElapsed(LauncherSubsystemConstants.kShootSpinUpSeconds)) {
      m_kickerSubsystem.RunKickerMM();
      m_conveyorSubsystem.RunConveyorMM();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_launcherSubsystem.StopLauncherMM();
    m_kickerSubsystem.StopKickerMM();
    m_conveyorSubsystem.StopConveyorMM();
    m_launcherSubsystem.RetractHoodMM();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double getHoodRotationsFromHoodAngleDeg(double hoodAngleDeg) {
    // Linear map  [kHoodMinAngleDeg, kHoodMaxAngleDeg] → [kHoodMinRot, kHoodMaxRot]
    double fraction = (hoodAngleDeg - LauncherSubsystemConstants.kHoodMinAngleDeg)
        / (LauncherSubsystemConstants.kHoodMaxAngleDeg - LauncherSubsystemConstants.kHoodMinAngleDeg);
    return LauncherSubsystemConstants.kHoodMinRot
        + fraction * (LauncherSubsystemConstants.kHoodMaxRot - LauncherSubsystemConstants.kHoodMinRot);
  }

  /**
   * Recomputes m_hoodPos and m_speed from the current robot state using ShootingArc,
   * and publishes both drag and no-drag solutions to SmartDashboard.
   */
  private void updateShootingParams() {
    Pose2d        robotPose     = m_poseSupplier.get();
    Translation2d robotVelocity = m_velocitySupplier.get();
    double exitSpeed = TurretSubsystemConstants.ballSpeed;

    // ── Drag-compensated solver (ACTIVE) ─────────────────────────────────────
    ShootingArc.Shot dragShot = m_shootingArc.solveDragShotWithLead(
        robotPose, robotVelocity, null, Optional.of(exitSpeed), false);

    SmartDashboard.putNumber("Launch/Drag/PitchDeg",    Math.toDegrees(dragShot.pitchRad()));
    SmartDashboard.putNumber("Launch/Drag/YawDeg",      Math.toDegrees(dragShot.yawFieldRad()));
    SmartDashboard.putNumber("Launch/Drag/FlightTimeS", dragShot.flightTimeS());
    SmartDashboard.putNumber("Launch/Drag/DistanceM",   dragShot.distanceM());
    SmartDashboard.putBoolean("Launch/Drag/OK",         dragShot.ok());

    // ── No-drag solver (COMMENTED OUT) ───────────────────────────────────────
    // ShootingArc.Shot noDragShot = m_shootingArc.solveNoDragWithLead(
    //     robotPose, robotVelocity, new Translation2d(), exitSpeed, false);
    //
    // SmartDashboard.putNumber("Launch/NoDrag/PitchDeg",    Math.toDegrees(noDragShot.pitchRad()));
    // SmartDashboard.putNumber("Launch/NoDrag/YawDeg",      Math.toDegrees(noDragShot.yawFieldRad()));
    // SmartDashboard.putNumber("Launch/NoDrag/FlightTimeS", noDragShot.flightTimeS());
    // SmartDashboard.putNumber("Launch/NoDrag/DistanceM",   noDragShot.distanceM());
    // SmartDashboard.putBoolean("Launch/NoDrag/OK",         noDragShot.ok());

    // ── Hood & speed from the active shot ────────────────────────────────────
    double launchAngleDeg = Math.toDegrees(dragShot.pitchRad());
    launchAngleDeg = Math.max(LauncherSubsystemConstants.kHoodMinAngleDeg,
                     Math.min(LauncherSubsystemConstants.kHoodMaxAngleDeg, launchAngleDeg));

    double fraction = (launchAngleDeg - LauncherSubsystemConstants.kHoodMinAngleDeg)
        / (LauncherSubsystemConstants.kHoodMaxAngleDeg - LauncherSubsystemConstants.kHoodMinAngleDeg);
    m_hoodPos = LauncherSubsystemConstants.kHoodMinRot
        + fraction * (LauncherSubsystemConstants.kHoodMaxRot - LauncherSubsystemConstants.kHoodMinRot);

    m_speed = exitSpeed / (Math.PI * LauncherSubsystemConstants.kLauncherWheelDiameterM);

    SmartDashboard.putNumber("Launch/CalculatedHoodAngleDeg", launchAngleDeg);
    SmartDashboard.putNumber("Launch/CalculatedHoodRot", m_hoodPos);
    SmartDashboard.putNumber("Launch/CalculatedLauncherRPS", m_speed);
  }
}

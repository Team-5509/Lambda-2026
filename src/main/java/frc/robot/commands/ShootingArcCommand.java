// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Algorithms.ShootingArc;
import frc.robot.Constants.Constants.LauncherSubsystemConstants;
import frc.robot.Constants.Constants.TurretSubsystemConstants;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Background command that continuously computes the shooting arc solution and
 * keeps the launcher spun up and hood positioned, ready to fire at any time.
 *
 * <p>Set this as the default command for {@link LauncherSubsystem} so it runs
 * perpetually. When the driver presses the fire trigger, {@link FireCommand}
 * (which only requires kicker + conveyor) can fire immediately without waiting
 * for spin-up.
 */
public class ShootingArcCommand extends Command {

  private final LauncherSubsystem m_launcherSubsystem;
  private final Supplier<Pose2d> m_poseSupplier;
  private final Supplier<Translation2d> m_velocitySupplier;

  private final ShootingArc m_shootingArc = new ShootingArc();

  private double m_speed   = LauncherSubsystemConstants.kHoodMinRot;
  private double m_hoodPos = LauncherSubsystemConstants.kHoodMinRot;

  /** The most recently computed shot solution — read by {@link FireCommand} if needed. */
  private ShootingArc.Shot m_lastShot = null;

  /**
   * @param launcherSubsystem  Launcher subsystem (hood + flywheel)
   * @param poseSupplier       Field-relative robot pose
   * @param velocitySupplier   Field-relative robot velocity in m/s
   */
  public ShootingArcCommand(
      LauncherSubsystem launcherSubsystem,
      Supplier<Pose2d> poseSupplier,
      Supplier<Translation2d> velocitySupplier) {
    m_launcherSubsystem = launcherSubsystem;
    m_poseSupplier      = poseSupplier;
    m_velocitySupplier  = velocitySupplier;

    addRequirements(launcherSubsystem);
  }

  @Override
  public void initialize() {
    // Nothing to initialize — computation starts immediately on the first execute()
  }

  @Override
  public void execute() {
    computeShootingParams();
    m_launcherSubsystem.extendHoodMM(m_hoodPos);
    m_launcherSubsystem.runLauncherMM(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_launcherSubsystem.StopLauncherMM();
    m_launcherSubsystem.RetractHoodMM();
  }

  /** Runs forever — this is a perpetual background command. */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Returns the most recently computed shot, or {@code null} before the first cycle. */
  public ShootingArc.Shot getLastShot() {
    return m_lastShot;
  }

  // ---------------------------------------------------------------------------
  // Internal helpers
  // ---------------------------------------------------------------------------

  private void computeShootingParams() {
    Pose2d        robotPose     = m_poseSupplier.get();
    Translation2d robotVelocity = m_velocitySupplier.get();
    double        exitSpeed     = TurretSubsystemConstants.ballSpeed;

    ShootingArc.Shot shot = m_shootingArc.solveDragShotWithLead(
        robotPose, robotVelocity, null, Optional.of(exitSpeed), false);

    m_lastShot = shot;

    SmartDashboard.putNumber("ShootingArc/BG/PitchDeg",    Math.toDegrees(shot.pitchRad()));
    SmartDashboard.putNumber("ShootingArc/BG/YawDeg",      Math.toDegrees(shot.yawFieldRad()));
    SmartDashboard.putNumber("ShootingArc/BG/FlightTimeS", shot.flightTimeS());
    SmartDashboard.putNumber("ShootingArc/BG/DistanceM",   shot.distanceM());
    SmartDashboard.putBoolean("ShootingArc/BG/OK",         shot.ok());

    double launchAngleDeg = Math.toDegrees(shot.pitchRad());
    launchAngleDeg = Math.max(LauncherSubsystemConstants.kHoodMinAngleDeg,
                     Math.min(LauncherSubsystemConstants.kHoodMaxAngleDeg, launchAngleDeg));

    double fraction = (launchAngleDeg - LauncherSubsystemConstants.kHoodMinAngleDeg)
        / (LauncherSubsystemConstants.kHoodMaxAngleDeg - LauncherSubsystemConstants.kHoodMinAngleDeg);
    m_hoodPos = LauncherSubsystemConstants.kHoodMinRot
        + fraction * (LauncherSubsystemConstants.kHoodMaxRot - LauncherSubsystemConstants.kHoodMinRot);

    m_speed = exitSpeed / (Math.PI * LauncherSubsystemConstants.kLauncherWheelDiameterM);

    SmartDashboard.putNumber("ShootingArc/BG/HoodAngleDeg", launchAngleDeg);
    SmartDashboard.putNumber("ShootingArc/BG/HoodRot",      m_hoodPos);
    SmartDashboard.putNumber("ShootingArc/BG/LauncherRPS",  m_speed);
  }
}

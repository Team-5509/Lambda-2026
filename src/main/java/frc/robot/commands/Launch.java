// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.function.Supplier;

import javax.xml.validation.SchemaFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
public class Launch extends Command {
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final LauncherSubsystem m_launcherSubsystem;
  private final KickerSubsystem   m_kickerSubsystem;
  private final Supplier<Pose2d>        m_poseSupplier;
  private final Supplier<Translation2d> m_velocitySupplier; // field-relative, m/s

  private final Timer m_timer = new Timer();

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
  public Launch(
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
   * Recomputes m_hoodPos and m_speed from the current robot state.
   *
   * Lead compensation (mirrors TrackFieldPoseCommand.aimFieldRelativeWithPrediction):
   *   1. Estimate flight time  =  current_distance / ballSpeed
   *   2. Predict shooter position at end of flight  =  shooterPos + velocity × flightTime
   *   3. Use the distance from that predicted position to the hub as the effective range
   *      in the ballistic formula, so the hood is set for where the ball will actually travel.
   *
   * Ballistic formula (low-angle solution, fixed ball speed):
   *   tan(θ) = (v² − √(v⁴ − g·(g·dx² + 2·dz·v²))) / (g·dx)
   */
  private void updateShootingParams() {
    Pose2d       robotPose     = m_poseSupplier.get();
    Translation2d robotVelocity = m_velocitySupplier.get();

    // Mirror TrackFieldPoseCommand target selection: hub when on own side, home otherwise
    Translation2d robotTranslation = robotPose.getTranslation();
    boolean isBlue = DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    Translation2d hub;
    double dz;
    if (isBlue && robotTranslation.getX() < TurretSubsystemConstants.blueLineZone) {
      hub = TurretSubsystemConstants.blueHubPose;
      dz  = TurretSubsystemConstants.dz;
    } else if (!isBlue && robotTranslation.getX() > TurretSubsystemConstants.redLineZone) {
      hub = TurretSubsystemConstants.redHubPose;
      dz  = TurretSubsystemConstants.dz;
    } else {
      var homePoses = isBlue
          ? TurretSubsystemConstants.blueHomePoses
          : TurretSubsystemConstants.redHomePoses;
      hub = homePoses.stream()
          .min((a, b) -> Double.compare(a.getDistance(robotTranslation), b.getDistance(robotTranslation)))
          .orElse(isBlue ? TurretSubsystemConstants.blueHubPose : TurretSubsystemConstants.redHubPose);
      dz  = TurretSubsystemConstants.homeDz;
    }

    // Shooter is offset from robot centre; rotate offset by current heading
    Translation2d shooterPos = robotPose.getTranslation().plus(
        TurretSubsystemConstants.shooterOffsetRobot.rotateBy(robotPose.getRotation()));

    // ── Lead compensation ────────────────────────────────────────────────────
    double dx0       = shooterPos.getDistance(hub);
    double flightTime = (dx0 > 0.01) ? dx0 / TurretSubsystemConstants.ballSpeed : 0.0;

    // Where the shooter will be when the ball arrives at the target
    Translation2d predictedShooterPos = shooterPos.plus(robotVelocity.times(flightTime));

    // Effective horizontal distance used for the ballistic calculation
    double dx = predictedShooterPos.getDistance(hub);

    // ── Hood angle from projectile-motion physics ────────────────────────────
    double v  = TurretSubsystemConstants.ballSpeed; // m/s
    double g  = TurretSubsystemConstants.g;         // m/s²
    double v2 = v * v;

    double launchAngleDeg;
    if (dx < 0.01) {
      // Robot is essentially on top of the hub — use minimum angle
      launchAngleDeg = LauncherSubsystemConstants.kHoodMinAngleDeg;
    } else {
      double discriminant = v2 * v2 - g * (g * dx * dx + 2.0 * dz * v2);
      if (discriminant < 0.0) {
        // Target is out of range at this ball speed — aim as steeply as possible
        launchAngleDeg = LauncherSubsystemConstants.kHoodMaxAngleDeg;
      } else {
        launchAngleDeg = Math.toDegrees(Math.atan((v2 - Math.sqrt(discriminant)) / (g * dx)));
      }
    }

    // Clamp to the physical hood travel range
    launchAngleDeg = Math.max(LauncherSubsystemConstants.kHoodMinAngleDeg,
                     Math.min(LauncherSubsystemConstants.kHoodMaxAngleDeg, launchAngleDeg));

    // Linear map  [kHoodMinAngleDeg, kHoodMaxAngleDeg] → [kHoodMinRot, kHoodMaxRot]
    // TODO: Calibrate kHoodMinAngleDeg and kHoodMaxAngleDeg on the robot.
    double fraction = (launchAngleDeg - LauncherSubsystemConstants.kHoodMinAngleDeg)
        / (LauncherSubsystemConstants.kHoodMaxAngleDeg - LauncherSubsystemConstants.kHoodMinAngleDeg);
    m_hoodPos = LauncherSubsystemConstants.kHoodMinRot
        + fraction * (LauncherSubsystemConstants.kHoodMaxRot - LauncherSubsystemConstants.kHoodMinRot);

    // ── Launcher speed ───────────────────────────────────────────────────────
    // RPS = exit_speed / wheel_circumference = v / (π · diameter)
    m_speed = v / (Math.PI * LauncherSubsystemConstants.kLauncherWheelDiameterM);
    SmartDashboard.putNumber("Launch/CalculatedHoodAngleDeg", launchAngleDeg);
    SmartDashboard.putNumber("Launch/CalculatedHoodRot", m_hoodPos);
    SmartDashboard.putNumber("Launch/CalculatedLauncherRPS", m_speed);
  }
}

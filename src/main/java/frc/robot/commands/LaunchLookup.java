// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
 * Spins up the launcher and feeds the ball using an interpolating lookup table.
 *
 * Hood position and launcher speed are looked up from measured/tuned tables
 * keyed on effective distance (meters) to the hub. Linear interpolation is
 * used between table entries; values outside the table range are clamped to
 * the nearest entry.
 *
 * Lead compensation accounts for robot motion during ball flight:
 *   predictedPos = shooterPos + velocity × flightTime
 *   effectiveDist = distance(predictedPos, hub)
 *
 * To tune: shoot from several known distances and record the hood rotation and
 * launcher speed that produce the best results, then update HOOD_TABLE and
 * SPEED_TABLE below.
 */
public class LaunchLookup extends Command {

  // ── Lookup tables (distance m → output) ─────────────────────────────────
  // TODO: Replace placeholder values with data measured on the actual robot.

  // Hub target tables
  private static final InterpolatingDoubleTreeMap HOOD_TABLE  = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap SPEED_TABLE = new InterpolatingDoubleTreeMap();

  // Home target tables (different goal height → different trajectory)
  // TODO: Tune these values by shooting at the home goal from known distances.
  private static final InterpolatingDoubleTreeMap HOOD_TABLE_HOME  = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap SPEED_TABLE_HOME = new InterpolatingDoubleTreeMap();

  static {
    // Hub: distance (m) → hood position (mechanism rotations)
    HOOD_TABLE.put(1.5, 0.40);
    HOOD_TABLE.put(2.5, 0.65);
    HOOD_TABLE.put(3.5, 0.95);
    HOOD_TABLE.put(4.5, 1.20);
    HOOD_TABLE.put(5.5, 1.45);
    HOOD_TABLE.put(6.5, 1.65);

    // Hub: distance (m) → launcher speed (RPS)
    SPEED_TABLE.put(1.5, 42.0);
    SPEED_TABLE.put(2.5, 55.0);
    SPEED_TABLE.put(3.5, 68.0);
    SPEED_TABLE.put(4.5, 78.0);
    SPEED_TABLE.put(5.5, 88.0);
    SPEED_TABLE.put(6.5, 96.0);

    // Home: distance (m) → hood position (mechanism rotations)
    // TODO: Replace placeholder values with data measured at the home goal.
    HOOD_TABLE_HOME.put(1.5, 0.35);
    HOOD_TABLE_HOME.put(2.5, 0.55);
    HOOD_TABLE_HOME.put(3.5, 0.80);
    HOOD_TABLE_HOME.put(4.5, 1.05);
    HOOD_TABLE_HOME.put(5.5, 1.30);
    HOOD_TABLE_HOME.put(6.5, 1.50);

    // Home: distance (m) → launcher speed (RPS)
    // TODO: Replace placeholder values with data measured at the home goal.
    SPEED_TABLE_HOME.put(1.5, 38.0);
    SPEED_TABLE_HOME.put(2.5, 50.0);
    SPEED_TABLE_HOME.put(3.5, 62.0);
    SPEED_TABLE_HOME.put(4.5, 72.0);
    SPEED_TABLE_HOME.put(5.5, 82.0);
    SPEED_TABLE_HOME.put(6.5, 90.0);
  }

  // ── Motion lead compensation constant ────────────────────────────────────
  // Approximate average ball speed used to estimate in-flight time.
  // flightTime = distance / kApproxBallSpeedMPS
  // The robot's current velocity is projected forward by flightTime so the
  // lookup uses where the shooter will actually be when the ball reaches the
  // (static) hub, rather than where it is when the trigger is pulled.
  // TODO: Replace with a measured value or a per-distance lookup if needed.
  private static final double kApproxBallSpeedMPS = 20.0; // m/s

  // ── Subsystems & suppliers ───────────────────────────────────────────────
  private final ConveyorSubsystem       m_conveyorSubsystem;
  private final LauncherSubsystem       m_launcherSubsystem;
  private final KickerSubsystem         m_kickerSubsystem;
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
   * @param poseSupplier       Field-relative robot pose
   * @param velocitySupplier   Field-relative robot velocity in m/s
   */
  public LaunchLookup(
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

    m_launcherSubsystem.ExtendHoodMM(() -> m_hoodPos);
    m_launcherSubsystem.RunLauncherMM(() -> m_speed);

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

  /**
   * Recomputes m_hoodPos and m_speed using lead-compensated distance and the
   * lookup tables.
   *
   * Mirrors TrackFieldPoseCommand target selection:
   *   - Hub tables when the robot is on its own side of the field.
   *   - Home tables when the robot has crossed to the opponent's side.
   *
   * Lead compensation steps:
   *   1. Compute current distance from shooter to target.
   *   2. Estimate ball flight time = currentDist / kApproxBallSpeedMPS.
   *   3. Predicted shooter position = shooterPos + velocity × flightTime.
   *   4. Effective distance = distance(predictedPos, target).
   *   5. Look up hood position and launcher speed from the appropriate tables.
   */
  private void updateShootingParams() {
    Pose2d        robotPose     = m_poseSupplier.get();
    Translation2d robotVelocity = m_velocitySupplier.get();

    // Mirror TrackFieldPoseCommand target selection: hub when on own side, home otherwise
    Translation2d robotTranslation = robotPose.getTranslation();
    boolean isBlue = DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    Translation2d hub;
    InterpolatingDoubleTreeMap hoodTable;
    InterpolatingDoubleTreeMap speedTable;
    if (isBlue && robotTranslation.getX() < TurretSubsystemConstants.blueLineZone) {
      hub        = TurretSubsystemConstants.blueHubPose;
      hoodTable  = HOOD_TABLE;
      speedTable = SPEED_TABLE;
    } else if (!isBlue && robotTranslation.getX() > TurretSubsystemConstants.redLineZone) {
      hub        = TurretSubsystemConstants.redHubPose;
      hoodTable  = HOOD_TABLE;
      speedTable = SPEED_TABLE;
    } else {
      var homePoses = isBlue
          ? TurretSubsystemConstants.blueHomePoses
          : TurretSubsystemConstants.redHomePoses;
      hub = homePoses.stream()
          .min((a, b) -> Double.compare(a.getDistance(robotTranslation), b.getDistance(robotTranslation)))
          .orElse(isBlue ? TurretSubsystemConstants.blueHubPose : TurretSubsystemConstants.redHubPose);
      hoodTable  = HOOD_TABLE_HOME;
      speedTable = SPEED_TABLE_HOME;
    }

    // Shooter is offset from robot centre; rotate offset by current heading
    Translation2d shooterPos = robotPose.getTranslation().plus(
        TurretSubsystemConstants.shooterOffsetRobot.rotateBy(robotPose.getRotation()));

    // ── Motion lead compensation ─────────────────────────────────────────────
    double currentDist = shooterPos.getDistance(hub);
    double flightTime  = (currentDist > 0.01) ? currentDist / kApproxBallSpeedMPS : 0.0;

    Translation2d predictedShooterPos = shooterPos.plus(robotVelocity.times(flightTime));
    double effectiveDist = predictedShooterPos.getDistance(hub);

    // ── Table lookup ─────────────────────────────────────────────────────────
    // InterpolatingDoubleTreeMap clamps to the min/max table entry when the
    // input is outside the measured range.
    m_hoodPos = hoodTable.get(effectiveDist);
    m_speed   = speedTable.get(effectiveDist);
    SmartDashboard.putNumber("LaunchLookup/CalculatedHoodRot", m_hoodPos);
    SmartDashboard.putNumber("LaunchLookup/CalculatedLauncherRPS", m_speed);
  }
}


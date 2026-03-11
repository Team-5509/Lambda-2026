package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.LauncherSubsystemConstants;
import frc.robot.Constants.Constants.TurretSubsystemConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Tuning tool for building the shooting lookup table.
 *
 * <p>Drive to a known position, adjust hood and speed on SmartDashboard,
 * press "Save Entry" to record it. All saved entries are published over
 * NetworkTables as parallel arrays so you can copy them into
 * {@link LaunchLookup}.
 *
 * <h3>SmartDashboard controls:</h3>
 * <ul>
 *   <li><b>Tuner/HoodRot</b> – hood position (mechanism rotations)</li>
 *   <li><b>Tuner/LauncherRPS</b> – launcher speed (RPS)</li>
 *   <li><b>Tuner/SaveEntry</b> – boolean button: save current entry</li>
 *   <li><b>Tuner/ClearAll</b> – boolean button: wipe all entries</li>
 *   <li><b>Tuner/AcceptShot</b> – boolean button: hold to spin up and shoot</li>
 * </ul>
 *
 * <h3>NetworkTables output ("ShootingTuner" table):</h3>
 * <ul>
 *   <li><b>Distances</b> – double[] of saved distances (m)</li>
 *   <li><b>HoodPositions</b> – double[] of saved hood rotations</li>
 *   <li><b>LauncherSpeeds</b> – double[] of saved launcher RPS</li>
 * </ul>
 */
public class ShootingLookupTuner extends Command {

  // ── Saved lookup entries ──────────────────────────────────────────────────
  private final ArrayList<Double> m_distances = new ArrayList<>();
  private final ArrayList<Double> m_hoodPositions = new ArrayList<>();
  private final ArrayList<Double> m_launcherSpeeds = new ArrayList<>();

  // ── NetworkTables publishers (send arrays to computer) ────────────────────
  private final NetworkTable m_nt =
      NetworkTableInstance.getDefault().getTable("ShootingTuner");
  private final DoubleArrayPublisher m_distPub =
      m_nt.getDoubleArrayTopic("Distances").publish();
  private final DoubleArrayPublisher m_hoodPub =
      m_nt.getDoubleArrayTopic("HoodPositions").publish();
  private final DoubleArrayPublisher m_speedPub =
      m_nt.getDoubleArrayTopic("LauncherSpeeds").publish();

  // ── Subsystems ────────────────────────────────────────────────────────────
  private final LauncherSubsystem m_launcher;
  private final KickerSubsystem m_kicker;
  private final ConveyorSubsystem m_conveyor;
  private final Supplier<Pose2d> m_poseSupplier;

  private boolean m_shooting = false;
  private int m_shootTicks = 0;

  /**
   * @param launcher      Launcher subsystem (hood + flywheel)
   * @param kicker        Kicker subsystem (trigger)
   * @param conveyor      Conveyor subsystem (ball feed)
   * @param poseSupplier  Current field-relative robot pose
   */
  public ShootingLookupTuner(
      LauncherSubsystem launcher,
      KickerSubsystem kicker,
      ConveyorSubsystem conveyor,
      Supplier<Pose2d> poseSupplier) {
    m_launcher = launcher;
    m_kicker = kicker;
    m_conveyor = conveyor;
    m_poseSupplier = poseSupplier;

    // Only require launcher — kicker/conveyor are controlled via instant
    // commands that we schedule on-demand so they don't conflict.
    addRequirements(launcher);
  }

  @Override
  public void initialize() {
    m_shooting = false;
    m_shootTicks = 0;

    // Seed SmartDashboard sliders if they don't already exist
    if (!SmartDashboard.containsKey("Tuner/HoodRot")) {
      SmartDashboard.putNumber("Tuner/HoodRot", 0.50);
    }
    if (!SmartDashboard.containsKey("Tuner/LauncherRPS")) {
      SmartDashboard.putNumber("Tuner/LauncherRPS", 50.0);
    }
    SmartDashboard.putBoolean("Tuner/SaveEntry", false);
    SmartDashboard.putBoolean("Tuner/ClearAll", false);
    SmartDashboard.putBoolean("Tuner/AcceptShot", false);
  }

  @Override
  public void execute() {
    // ── 1. Compute distance to target ─────────────────────────────────────
    Pose2d robotPose = m_poseSupplier.get();
    Translation2d robotPos = robotPose.getTranslation();

    boolean isBlue = DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    Translation2d hub = isBlue
        ? TurretSubsystemConstants.blueHubPose
        : TurretSubsystemConstants.redHubPose;

    // Shooter offset from robot centre
    Translation2d shooterPos = robotPos.plus(
        TurretSubsystemConstants.shooterOffsetRobot.rotateBy(robotPose.getRotation()));
    double distance = shooterPos.getDistance(hub);

    SmartDashboard.putNumber("Tuner/DistanceToTarget", distance);
    SmartDashboard.putNumber("Tuner/RobotX", robotPos.getX());
    SmartDashboard.putNumber("Tuner/RobotY", robotPos.getY());

    // ── 2. Read operator-adjustable values ────────────────────────────────
    double hoodRot = SmartDashboard.getNumber("Tuner/HoodRot", 0.50);
    double launcherRPS = SmartDashboard.getNumber("Tuner/LauncherRPS", 50.0);

    // ── 3. Handle "Save Entry" button ─────────────────────────────────────
    if (SmartDashboard.getBoolean("Tuner/SaveEntry", false)) {
      SmartDashboard.putBoolean("Tuner/SaveEntry", false);
      m_distances.add(distance);
      m_hoodPositions.add(hoodRot);
      m_launcherSpeeds.add(launcherRPS);
      publishArrays();
    }

    // ── 4. Handle "Clear All" button ──────────────────────────────────────
    if (SmartDashboard.getBoolean("Tuner/ClearAll", false)) {
      SmartDashboard.putBoolean("Tuner/ClearAll", false);
      m_distances.clear();
      m_hoodPositions.clear();
      m_launcherSpeeds.clear();
      publishArrays();
    }

    // ── 5. Handle "Accept Shot" ───────────────────────────────────────────
    boolean wantsShot = SmartDashboard.getBoolean("Tuner/AcceptShot", false);
    if (wantsShot) {
      if (!m_shooting) {
        m_shooting = true;
        m_shootTicks = 0;
      }

      // Position hood and spin up flywheel (direct method calls on launcher)
      m_launcher.extendHoodMM(hoodRot);
      m_launcher.runLauncherMM(launcherRPS);

      // After spin-up wait, feed the ball via kicker + conveyor
      m_shootTicks++;
      int spinUpTicks = (int) (LauncherSubsystemConstants.kShootSpinUpSeconds * 50);
      if (m_shootTicks > spinUpTicks) {
        m_kicker.RunKickerMM().schedule();
        m_conveyor.RunConveyorMM().schedule();
      }

      SmartDashboard.putBoolean("Tuner/Shooting", true);
    } else {
      if (m_shooting) {
        // Operator released the shot button — stop everything
        m_launcher.runLauncherMM(0);
        m_kicker.StopKickerMM().schedule();
        m_conveyor.StopConveyorMM().schedule();
        m_shooting = false;
        m_shootTicks = 0;
      }
      SmartDashboard.putBoolean("Tuner/Shooting", false);
    }

    // ── 6. Show table info ────────────────────────────────────────────────
    SmartDashboard.putNumber("Tuner/EntryCount", m_distances.size());
    SmartDashboard.putString("Tuner/TablePreview", buildTableString());
  }

  @Override
  public void end(boolean interrupted) {
    m_launcher.runLauncherMM(0);
    m_kicker.StopKickerMM().schedule();
    m_conveyor.StopConveyorMM().schedule();
    SmartDashboard.putBoolean("Tuner/Shooting", false);
    SmartDashboard.putBoolean("Tuner/AcceptShot", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // ── Helpers ─────────────────────────────────────────────────────────────

  /** Publish the saved arrays over NetworkTables. */
  private void publishArrays() {
    m_distPub.set(toDoubleArray(m_distances));
    m_hoodPub.set(toDoubleArray(m_hoodPositions));
    m_speedPub.set(toDoubleArray(m_launcherSpeeds));
    SmartDashboard.putNumber("Tuner/EntryCount", m_distances.size());
  }

  /** Human-readable table string for quick dashboard preview. */
  private String buildTableString() {
    if (m_distances.isEmpty()) return "(empty)";
    StringBuilder sb = new StringBuilder();
    sb.append("Dist(m) | Hood(rot) | Speed(RPS)\n");
    for (int i = 0; i < m_distances.size(); i++) {
      sb.append(String.format("%.2f    | %.3f     | %.1f\n",
          m_distances.get(i), m_hoodPositions.get(i), m_launcherSpeeds.get(i)));
    }
    return sb.toString();
  }

  private static double[] toDoubleArray(ArrayList<Double> list) {
    double[] arr = new double[list.size()];
    for (int i = 0; i < list.size(); i++) {
      arr[i] = list.get(i);
    }
    return arr;
  }
}

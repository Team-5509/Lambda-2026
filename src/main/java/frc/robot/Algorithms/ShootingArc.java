package frc.robot.Algorithms;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.LauncherSubsystemConstants;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

import edu.wpi.first.math.filter.LinearFilter;

public class ShootingArc {

  // Hood angle limits in radians (derived from Constants)
  private static final double HOOD_MIN_RAD = Math.toRadians(LauncherSubsystemConstants.kHoodMinAngleDeg);
  private static final double HOOD_MAX_RAD = Math.toRadians(LauncherSubsystemConstants.kHoodMaxAngleDeg);

  private static final Translation2d ZERO_VEC = new Translation2d();

  public class FieldAccelEstimator {
    private Translation2d lastV = new Translation2d();
    private double lastT = Timer.getFPGATimestamp();

    // Simple low-pass to reduce noise (tune time constant)
    private final LinearFilter axFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter ayFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public Translation2d update(Translation2d vField) {
      double now = Timer.getFPGATimestamp();
      double dt = now - lastT;
      lastT = now;

      if (dt <= 1e-3)
        return new Translation2d();

      double ax = (vField.getX() - lastV.getX()) / dt;
      double ay = (vField.getY() - lastV.getY()) / dt;
      lastV = vField;

      ax = axFilter.calculate(ax);
      ay = ayFilter.calculate(ay);

      return new Translation2d(ax, ay); // field-relative m/s^2
    }
  }

  private final double hubX;
  private final double hubY;

  public ShootingArc() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      hubX = Constants.TurretSubsystemConstants.redHubPose.getX();
      hubY = Constants.TurretSubsystemConstants.redHubPose.getY();
    } else {
      hubX = Constants.TurretSubsystemConstants.blueHubPose.getX();
      hubY = Constants.TurretSubsystemConstants.blueHubPose.getY();
    }
  }

  public Pose2d getShooterPosition(Pose2d robotPose) {
    return robotPose.transformBy(
        new Transform2d(Constants.TurretSubsystemConstants.shooterOffsetRobot, robotPose.getRotation()));
  }

  public Translation2d getHubTranslation() {
    return new Translation2d(hubX, hubY);
  }

  public double getDistanceToHub(Pose2d robotPose) {
    double dx = hubX - robotPose.getX();
    double dy = hubY - robotPose.getY();
    double distance = Math.sqrt(dx * dx + dy * dy);
    SmartDashboard.putNumber("ShootingArc/DistanceToGoal", distance);
    return distance;
  }

  public double getYawToHub(Pose2d robotPose) {
    double dx = hubX - robotPose.getX();
    double dy = hubY - robotPose.getY();
    double angle = Math.atan2(dy, dx);
    SmartDashboard.putNumber("ShootingArc/YawToGoalDeg", Math.toDegrees(angle));
    return angle;
  }

  // -------------------------------------------------------------------------
  // Shot result record
  // -------------------------------------------------------------------------

  public static record Shot(
      double pitchRad,
      double yawFieldRad,
      Optional<Double> launcherSpeedMps,
      double flightTimeS,
      double distanceM,
      boolean ok) {
  }

  // -------------------------------------------------------------------------
  // No-drag solver (with lead)
  // -------------------------------------------------------------------------

  /** Full signature with all parameters. */
  public Shot solveNoDragWithLead(
      Pose2d robotPose,
      Translation2d robotVelField,
      Translation2d robotAccelField,
      double exitSpeedMps,
      boolean preferHighArc) {
    Translation2d goal = getHubTranslation();
    Translation2d robotPos = getShooterPosition(robotPose).getTranslation();

    Translation2d r0 = goal.minus(robotPos);
    double t = r0.getNorm() / Math.max(exitSpeedMps, 0.1);

    double yaw = Math.atan2(r0.getY(), r0.getX());
    double pitch = 0.0;

    for (int i = 0; i < 5; i++) {
      Translation2d robotDelta = robotVelField.times(t).plus(robotAccelField.times(0.5 * t * t));
      Translation2d r = r0.minus(robotDelta);
      yaw = Math.atan2(r.getY(), r.getX());

      double R = r.getNorm();
      var pitchOpt = solvePitchNoDrag(R, Constants.TurretSubsystemConstants.dz, exitSpeedMps, preferHighArc);

      if (pitchOpt == null) {
        SmartDashboard.putBoolean("ShootingArc/HasSolution", false);
        return new Shot(0.0, yaw, Optional.of(exitSpeedMps), t, R, false);
      }

      pitch = pitchOpt;

      double vHoriz = exitSpeedMps * Math.cos(pitch);
      t = R / Math.max(vHoriz, 0.1);
    }

    Translation2d robotDeltaFinal = robotVelField.times(t).plus(robotAccelField.times(0.5 * t * t));
    double Rfinal = r0.minus(robotDeltaFinal).getNorm();

    SmartDashboard.putBoolean("ShootingArc/HasSolution", true);
    SmartDashboard.putNumber("ShootingArc/FlightTimeS", t);
    SmartDashboard.putNumber("ShootingArc/YawDeg", Math.toDegrees(yaw));
    SmartDashboard.putNumber("ShootingArc/PitchDeg", Math.toDegrees(pitch));

    return new Shot(pitch, yaw, Optional.of(exitSpeedMps), t, Rfinal, true);
  }

  /** No velocity/accel — stationary robot. */
  public Shot solveNoDragWithLead(Pose2d robotPose, double exitSpeedMps, boolean preferHighArc) {
    return solveNoDragWithLead(robotPose, ZERO_VEC, ZERO_VEC, exitSpeedMps, preferHighArc);
  }

  /**
   * Ballistic pitch solution without drag.
   * Clamps result to hood angle limits [kHoodMinAngleDeg, kHoodMaxAngleDeg].
   * Returns pitch angle in radians, or null if impossible within limits.
   */
  private static Double solvePitchNoDrag(double R, double dz, double v, boolean highArc) {
    double g = Constants.TurretSubsystemConstants.g;
    double v2 = v * v;
    double v4 = v2 * v2;

    double disc = v4 - g * (g * R * R + 2.0 * dz * v2);
    if (disc < 0)
      return null;

    double sqrt = Math.sqrt(disc);
    double denom = g * R;

    double tanLow = (v2 - sqrt) / denom;
    double tanHigh = (v2 + sqrt) / denom;

    double low = Math.atan(tanLow);
    double high = Math.atan(tanHigh);

    double chosen = highArc ? high : low;

    // Clamp to hood limits; if outside range, no valid solution
    if (chosen < HOOD_MIN_RAD || chosen > HOOD_MAX_RAD) {
      // Try the other arc — it might be in range
      double other = highArc ? low : high;
      if (other >= HOOD_MIN_RAD && other <= HOOD_MAX_RAD) {
        return other;
      }
      return null;
    }

    return chosen;
  }

  // -------------------------------------------------------------------------
  // Drag-compensated solver
  // -------------------------------------------------------------------------

  /** Simple drag shot — stationary robot, known exit speed. */
  public Shot solveDragShot(Pose2d robotPose, double exitSpeedMps) {
    Pose2d shooterPose = getShooterPosition(robotPose);
    double distanceM = getDistanceToHub(shooterPose);
    return solvePitchWithQuadraticDrag(distanceM, Constants.TurretSubsystemConstants.dz, exitSpeedMps, 0.03, true);
  }

  /** Simple drag shot — stationary robot, auto-solve for exit speed. */
  public Shot solveDragShot(Pose2d robotPose) {
    Pose2d shooterPose = getShooterPosition(robotPose);
    double distanceM = getDistanceToHub(shooterPose);
    return solveSpeedAndPitchWithDrag(distanceM, Constants.TurretSubsystemConstants.dz, 0.03, true);
  }

  /**
   * Full drag-compensated shot solver with robot motion lead.
   * All parameters except robotPose and preferHighArc are optional:
   * - exitSpeedMps: if empty, auto-solves for minimum viable speed (capped at kMaxExitSpeedMps)
   * - robotVelField / robotAccelField: if null, defaults to zero (stationary robot)
   *
   * @param robotPose       Current field pose of the robot
   * @param robotVelField   Field-relative velocity (m/s), or null for stationary
   * @param robotAccelField Field-relative acceleration (m/s²), or null for stationary
   * @param exitSpeedMps    Ball exit speed (m/s), or empty to auto-solve
   * @param preferHighArc   false = low arc, true = high arc
   */
  public Shot solveDragShotWithLead(
      Pose2d robotPose,
      Translation2d robotVelField,
      Translation2d robotAccelField,
      Optional<Double> exitSpeedMps,
      boolean preferHighArc) {

    Translation2d vel   = (robotVelField   != null) ? robotVelField   : ZERO_VEC;
    Translation2d accel = (robotAccelField != null) ? robotAccelField : ZERO_VEC;

    Translation2d goal       = getHubTranslation();
    Translation2d shooterPos = getShooterPosition(robotPose).getTranslation();
    Translation2d r0         = goal.minus(shooterPos);

    // If no exit speed provided, solve for it first
    double speed;
    if (exitSpeedMps.isPresent()) {
      speed = Math.min(exitSpeedMps.get(), LauncherSubsystemConstants.kMaxExitSpeedMps);
    } else {
      // Estimate lead-compensated distance for the speed solver
      double roughT = r0.getNorm() / 10.0;
      Translation2d roughDelta = vel.times(roughT).plus(accel.times(0.5 * roughT * roughT));
      double roughR = r0.minus(roughDelta).getNorm();

      Shot speedShot = solveSpeedAndPitchWithDrag(roughR, Constants.TurretSubsystemConstants.dz, 0.03, preferHighArc);
      if (!speedShot.ok() || speedShot.launcherSpeedMps().isEmpty()) {
        return new Shot(0.0, Math.atan2(r0.getY(), r0.getX()), Optional.empty(), 0, roughR, false);
      }
      speed = speedShot.launcherSpeedMps().get();
    }

    double t   = r0.getNorm() / Math.max(speed, 0.1);
    double yaw = Math.atan2(r0.getY(), r0.getX());
    double pitch = 0.0;

    for (int i = 0; i < 6; i++) {
      Translation2d robotDelta = vel.times(t)
          .plus(accel.times(0.5 * t * t));

      Translation2d r = r0.minus(robotDelta);
      yaw = Math.atan2(r.getY(), r.getX());
      double R = r.getNorm();

      Shot dragShot = solvePitchWithQuadraticDrag(R, Constants.TurretSubsystemConstants.dz, speed, 0.03, preferHighArc);

      if (!dragShot.ok()) {
        SmartDashboard.putBoolean("ShootingArc/HasSolution", false);
        return new Shot(0.0, yaw, Optional.of(speed), t, R, false);
      }

      pitch = dragShot.pitchRad();
      t     = dragShot.flightTimeS();

      SmartDashboard.putNumber("ShootingArc/Lead/Iter" + i + "/R",        R);
      SmartDashboard.putNumber("ShootingArc/Lead/Iter" + i + "/t",        t);
      SmartDashboard.putNumber("ShootingArc/Lead/Iter" + i + "/yawDeg",   Math.toDegrees(yaw));
      SmartDashboard.putNumber("ShootingArc/Lead/Iter" + i + "/pitchDeg", Math.toDegrees(pitch));
    }

    Translation2d robotDeltaFinal = vel.times(t).plus(accel.times(0.5 * t * t));
    double Rfinal = r0.minus(robotDeltaFinal).getNorm();

    SmartDashboard.putBoolean("ShootingArc/HasSolution", true);
    SmartDashboard.putNumber("ShootingArc/FlightTimeS", t);
    SmartDashboard.putNumber("ShootingArc/YawDeg",      Math.toDegrees(yaw));
    SmartDashboard.putNumber("ShootingArc/PitchDeg",    Math.toDegrees(pitch));

    return new Shot(pitch, yaw, Optional.of(speed), t, Rfinal, true);
  }

  /** Convenience: known exit speed, no robot motion. */
  public Shot solveDragShotWithLead(Pose2d robotPose, double exitSpeedMps, boolean preferHighArc) {
    return solveDragShotWithLead(robotPose, null, null, Optional.of(exitSpeedMps), preferHighArc);
  }

  /** Convenience: auto-solve speed, no robot motion. */
  public Shot solveDragShotWithLead(Pose2d robotPose, boolean preferHighArc) {
    return solveDragShotWithLead(robotPose, null, null, Optional.empty(), preferHighArc);
  }

  // -------------------------------------------------------------------------
  // Pitch solver with quadratic drag (bisection on angle)
  // -------------------------------------------------------------------------

  /**
   * Solve pitch angle with quadratic drag using bisection on a numeric simulation.
   * Search range is clamped to hood angle limits [kHoodMinAngleDeg, kHoodMaxAngleDeg].
   */
  public static Shot solvePitchWithQuadraticDrag(
      double R, double dz, double v0, double k, boolean highArc
  ) {
    // Clamp search range to physical hood limits
    double lo = HOOD_MIN_RAD;
    double hi = HOOD_MAX_RAD;

    // For high arc, bias initial bracket higher
    if (highArc) lo = Math.max(lo, Math.toRadians(25));

    Double yLo = simulateYAtX(R, dz, v0, lo, k);
    Double yHi = simulateYAtX(R, dz, v0, hi, k);
    if (yLo == null || yHi == null) return new Shot(0, 0, Optional.of(v0), 0, 0, false);

    double fLo = yLo;
    double fHi = yHi;

    // Need a sign change for bisection; if not, coarse-scan for closest angle.
    if (Math.signum(fLo) == Math.signum(fHi)) {
      double bestTheta = lo;
      double bestAbs = Math.abs(fLo);
      double bestT = 0;

      for (int i = 0; i <= 60; i++) {
        double th = lo + (hi - lo) * i / 60.0;
        SimResult sr = simulateToX(R, v0, th, k);
        if (!sr.ok) continue;
        double f = sr.y - dz;
        double af = Math.abs(f);
        if (af < bestAbs) {
          bestAbs = af;
          bestTheta = th;
          bestT = sr.t;
        }
      }
      return new Shot(bestTheta, 0, Optional.of(v0), bestT, 0, bestAbs < 0.10); // 10 cm tolerance
    }

    // Bisection
    double mid = 0;
    SimResult midRes = null;

    for (int iter = 0; iter < 30; iter++) {
      mid = 0.5 * (lo + hi);
      SimResult sr = simulateToX(R, v0, mid, k);
      if (!sr.ok) return new Shot(0, 0, Optional.of(v0), 0, 0, false);

      double fMid = sr.y - dz;
      midRes = sr;

      if (Math.abs(fMid) < 0.01) { // 1 cm
        return new Shot(mid, 0, Optional.of(v0), sr.t, 0, true);
      }

      if (Math.signum(fMid) == Math.signum(fLo)) {
        lo = mid;
        fLo = fMid;
      } else {
        hi = mid;
        fHi = fMid;
      }
    }

    if (midRes == null) return new Shot(0, 0, Optional.of(v0), 0, 0, false);
    return new Shot(mid, 0, Optional.of(v0), midRes.t, 0, true);
  }

  // -------------------------------------------------------------------------
  // Speed solver — find minimum exit speed for a valid shot within hood limits
  // -------------------------------------------------------------------------

  /**
   * Binary-search for the minimum exit speed that produces a valid shot
   * with pitch within hood angle limits. Caps at kMaxExitSpeedMps.
   *
   * @param R        Horizontal distance to target (m)
   * @param dz       Height difference: target - shooter (m)
   * @param k        Drag coefficient
   * @param highArc  Prefer high arc
   * @return Shot with launcherSpeedMps populated, or ok=false if unreachable
   */
  public static Shot solveSpeedAndPitchWithDrag(double R, double dz, double k, boolean highArc) {
    double maxSpeed = LauncherSubsystemConstants.kMaxExitSpeedMps;
    double speedLo = 1.0;   // minimum plausible speed
    double speedHi = maxSpeed;

    // First check if max speed can even reach the target
    Shot maxShot = solvePitchWithQuadraticDrag(R, dz, speedHi, k, highArc);
    if (!maxShot.ok()) {
      return new Shot(0, 0, Optional.empty(), 0, R, false);
    }

    // Binary search for minimum viable speed
    Shot bestShot = maxShot;
    for (int i = 0; i < 20; i++) {
      double mid = 0.5 * (speedLo + speedHi);
      Shot trial = solvePitchWithQuadraticDrag(R, dz, mid, k, highArc);
      if (trial.ok()) {
        bestShot = trial;
        speedHi = mid;
      } else {
        speedLo = mid;
      }
    }

    double solvedSpeed = 0.5 * (speedLo + speedHi);
    // Clamp to max
    solvedSpeed = Math.min(solvedSpeed, maxSpeed);

    return new Shot(
        bestShot.pitchRad(),
        bestShot.yawFieldRad(),
        Optional.of(solvedSpeed),
        bestShot.flightTimeS(),
        R,
        true);
  }

  // -------------------------------------------------------------------------
  // Simulation helpers
  // -------------------------------------------------------------------------

  private static class SimResult {
    final double y;
    final double t;
    final boolean ok;
    SimResult(double y, double t, boolean ok) { this.y = y; this.t = t; this.ok = ok; }
  }

  private static Double simulateYAtX(double R, double dz, double v0, double theta, double k) {
    SimResult sr = simulateToX(R, v0, theta, k);
    if (!sr.ok) return null;
    return sr.y - dz;
  }

  /**
   * Integrate 2D point-mass with quadratic drag until x >= R (or time/ground limit).
   * x = forward (m), y = up (m).
   */
  private static SimResult simulateToX(double R, double v0, double theta, double k) {
    final double g = 9.80665;
    final double dt = 0.005;  // 200 Hz sim
    final double tMax = 3.0;  // cap flight time

    double x = 0, y = 0;
    double vx = v0 * Math.cos(theta);
    double vy = v0 * Math.sin(theta);
    double t = 0;

    if (vx <= 0.1) return new SimResult(0, 0, false);

    while (t < tMax) {
      if (x >= R) return new SimResult(y, t, true);
      if (y < -0.2) return new SimResult(y, t, false);

      double speed = Math.hypot(vx, vy);
      double ax = -k * speed * vx;
      double ay = -g - k * speed * vy;

      // Semi-implicit Euler
      vx += ax * dt;
      vy += ay * dt;
      x  += vx * dt;
      y  += vy * dt;
      t  += dt;
    }

    return new SimResult(y, tMax, false);
  }
}

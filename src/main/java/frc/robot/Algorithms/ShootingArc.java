package frc.robot.Algorithms;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

import edu.wpi.first.math.filter.LinearFilter;

public class ShootingArc {

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

  // Field goal positions (meters). You already had inches->m, keep that idea.
  public static class Constants {
    public static final double BlueHubY = 158.32 * 0.0254;
    public static final double BlueHubX = 181.56 * 0.0254;
    public static final double RedHubY = 158.32 * 0.0254;
    public static final double RedHubX = 468.56 * 0.0254;

    public static final double g = 9.80665; // m/s^2

    // Measure these on the robot:
    public static final double shooterHeightM = 21.5 * 0.0254; // TODO: Measure this 21.5 in -> m
    public static final double goalHeightM = 72.0 * 0.0254; // 72 in -> m
    public static final double dz = goalHeightM - shooterHeightM;

    //TODO: Measure this from shooter exit to robot center 
    public static final Translation2d shooterOffsetRobot = new Translation2d(0.32, 0.18);
  }

  private final double hubX;
  private final double hubY;

  public ShootingArc() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      hubX = Constants.RedHubX;
      hubY = Constants.RedHubY;
    } else {
      hubX = Constants.BlueHubX;
      hubY = Constants.BlueHubY;
    }
  }

  /**
   * Get shooter position in field coordinates by applying robot pose + shooter offset.
   * @param robotPose
   * @return
   */
  public Pose2d getShooterPosition(Pose2d robotPose) {
    // Get shooter position in field coordinates by applying robot pose + shooter offset
    return robotPose.transformBy(
        new Transform2d(Constants.shooterOffsetRobot, robotPose.getRotation()));
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

  /**
   * Solve for yaw + pitch while compensating for robot translation during the
   * shot.
   *
   * @param robotPose       Field pose of robot now
   * @param robotVelField   Field-relative velocity of robot (m/s)
   * @param robotAccelField Field-relative acceleration of robot (m/s^2) (can be
   *                        0,0 if unknown)
   * @param exitSpeedMps    Ball exit speed from shooter (m/s)
   * @param preferHighArc   false = low arc, true = high arc (if feasible)
   */
  public Shot solveNoDragWithLead(
      Pose2d robotPose,
      Translation2d robotVelField,
      Translation2d robotAccelField,
      double exitSpeedMps,
      boolean preferHighArc) {
    Translation2d goal = getHubTranslation();
    Translation2d robotPos = getShooterPosition(robotPose).getTranslation();

    // Initial guess ignoring accel: time ≈ distance / speed
    Translation2d r0 = goal.minus(robotPos);
    double t = r0.getNorm() / Math.max(exitSpeedMps, 0.1);

    double yaw = Math.atan2(r0.getY(), r0.getX());
    double pitch = 0.0;

    // Iterate a few times (3–6 is plenty)
    for (int i = 0; i < 5; i++) {
      Translation2d robotDelta = robotVelField.times(t).plus(robotAccelField.times(0.5 * t * t));

      Translation2d r = r0.minus(robotDelta); // required displacement the ball must cover in field
      yaw = Math.atan2(r.getY(), r.getX());

      double R = r.getNorm(); // horizontal distance to cover (we're assuming flat field)
      var pitchOpt = solvePitchNoDrag(R, Constants.dz, exitSpeedMps, preferHighArc);

      if (pitchOpt == null) {
        // No real ballistic solution at this speed/distance
        SmartDashboard.putBoolean("ShootingArc/HasSolution", false);
        return new Shot(0.0, yaw, t, R, false);
      }

      pitch = pitchOpt;

      // Update time of flight from horizontal component:
      double vHoriz = exitSpeedMps * Math.cos(pitch);
      t = R / Math.max(vHoriz, 0.1);

      SmartDashboard.putNumber("ShootingArc/Iter" + i + "/R", R);
      SmartDashboard.putNumber("ShootingArc/Iter" + i + "/t", t);
      SmartDashboard.putNumber("ShootingArc/Iter" + i + "/yawDeg", Math.toDegrees(yaw));
      SmartDashboard.putNumber("ShootingArc/Iter" + i + "/pitchDeg", Math.toDegrees(pitch));
    }

    Translation2d robotDeltaFinal = robotVelField.times(t).plus(robotAccelField.times(0.5 * t * t));
    double Rfinal = r0.minus(robotDeltaFinal).getNorm();

    SmartDashboard.putBoolean("ShootingArc/HasSolution", true);
    SmartDashboard.putNumber("ShootingArc/FlightTimeS", t);
    SmartDashboard.putNumber("ShootingArc/YawDeg", Math.toDegrees(yaw));
    SmartDashboard.putNumber("ShootingArc/PitchDeg", Math.toDegrees(pitch));

    return new Shot(pitch, yaw, t, Rfinal, true);
  }

  /**
   * Ballistic pitch solution without drag.
   * Returns pitch angle in radians, or null if impossible.
   *
   * Formula:
   * tan(theta) = (v^2 ± sqrt(v^4 - g(gR^2 + 2dz v^2))) / (gR)
   */
  private static Double solvePitchNoDrag(double R, double dz, double v, boolean highArc) {
    double g = Constants.g;
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

    return highArc ? high : low;
  }


  // -------------------------------------------------------------------------
  // Drag-compensated solver
  // -------------------------------------------------------------------------

  public static record Shot(double pitchRad, double yawFieldRad, double flightTimeS, double distanceM, boolean ok) {}

  /**
   * Simplified drag shot solver using robot pose. Uses default drag (k=0.03) and Constants.dz.
   * Distance is measured from the shooter's translated position (not the robot center).
   */
  public Shot solveDragShot(Pose2d robotPose, double exitSpeedMps) {
    Pose2d shooterPose = getShooterPosition(robotPose);
    double distanceM = getDistanceToHub(shooterPose);
    return solvePitchWithQuadraticDrag(distanceM, Constants.dz, exitSpeedMps, 0.03, true);
  }

  /**
   * Drag-compensated shot solver with robot motion lead.
   * Combines quadratic-drag physics (via numeric sim) with iterative lead compensation
   * for robot velocity and acceleration during the ball's flight.
   *
   * <p>Each iteration:
   * <ol>
   *   <li>Projects the shooter position forward by {@code vel*t + 0.5*accel*t²}</li>
   *   <li>Computes the required ball displacement and yaw to the goal</li>
   *   <li>Solves drag pitch for that horizontal distance (gives a physically-accurate flight time)</li>
   *   <li>Uses that flight time in the next iteration</li>
   * </ol>
   * Converges in 4–6 iterations for typical FRC speeds.
   *
   * @param robotPose       Current field pose of the robot
   * @param robotVelField   Field-relative velocity of the robot (m/s)
   * @param robotAccelField Field-relative acceleration of the robot (m/s²); pass zero vector if unknown
   * @param exitSpeedMps    Ball exit speed from shooter (m/s)
   * @param preferHighArc   false = low arc, true = high arc
   */
  public Shot solveDragShotWithLead(
      Pose2d robotPose,
      Translation2d robotVelField,
      Translation2d robotAccelField,
      double exitSpeedMps,
      boolean preferHighArc) {

    Translation2d goal       = getHubTranslation();
    Translation2d shooterPos = getShooterPosition(robotPose).getTranslation();
    Translation2d r0         = goal.minus(shooterPos); // static displacement, no motion yet

    // Initial flight-time guess: distance / exit speed
    double t   = r0.getNorm() / Math.max(exitSpeedMps, 0.1);
    double yaw = Math.atan2(r0.getY(), r0.getX());
    double pitch = 0.0;

    for (int i = 0; i < 6; i++) {
      // Where the shooter will have moved by the time the ball arrives
      Translation2d robotDelta = robotVelField.times(t)
          .plus(robotAccelField.times(0.5 * t * t));

      // Displacement the ball must travel in field frame
      Translation2d r = r0.minus(robotDelta);
      yaw = Math.atan2(r.getY(), r.getX());
      double R = r.getNorm();

      // Solve drag pitch for this horizontal distance; reuse its sim flight time
      Shot dragShot = solvePitchWithQuadraticDrag(R, Constants.dz, exitSpeedMps, 0.03, preferHighArc);

      if (!dragShot.ok()) {
        SmartDashboard.putBoolean("ShootingArc/HasSolution", false);
        return new Shot(0.0, yaw, t, R, false);
      }

      pitch = dragShot.pitchRad();
      t     = dragShot.flightTimeS(); // physically accurate; drives convergence

      SmartDashboard.putNumber("ShootingArc/Lead/Iter" + i + "/R",        R);
      SmartDashboard.putNumber("ShootingArc/Lead/Iter" + i + "/t",        t);
      SmartDashboard.putNumber("ShootingArc/Lead/Iter" + i + "/yawDeg",   Math.toDegrees(yaw));
      SmartDashboard.putNumber("ShootingArc/Lead/Iter" + i + "/pitchDeg", Math.toDegrees(pitch));
    }

    Translation2d robotDeltaFinal = robotVelField.times(t).plus(robotAccelField.times(0.5 * t * t));
    double Rfinal = r0.minus(robotDeltaFinal).getNorm();

    SmartDashboard.putBoolean("ShootingArc/HasSolution", true);
    SmartDashboard.putNumber("ShootingArc/FlightTimeS", t);
    SmartDashboard.putNumber("ShootingArc/YawDeg",      Math.toDegrees(yaw));
    SmartDashboard.putNumber("ShootingArc/PitchDeg",    Math.toDegrees(pitch));

    return new Shot(pitch, yaw, t, Rfinal, true);
  }

  /**
   * Solve pitch angle with quadratic drag using bisection on a numeric simulation.
   *
   * @param R        Horizontal distance to target (m)
   * @param dz       Height difference: target - shooter (m)
   * @param v0       Ball exit speed (m/s)
   * @param k        Drag coefficient (kg/m, i.e. F_drag = k * |v| * v). Start with ~0.01–0.05.
   * @param highArc  false = low arc, true = high arc
   */
  public static Shot solvePitchWithQuadraticDrag(
      double R, double dz, double v0, double k, boolean highArc
  ) {
    // Search pitch range (avoid exactly 0 or 90)
    double lo = Math.toRadians(2);
    double hi = Math.toRadians(80);

    // For high arc, bias initial bracket higher
    if (highArc) lo = Math.toRadians(25);

    // If even the best angle can't reach, fail fast
    Double yLo = simulateYAtX(R, dz, v0, lo, k);
    Double yHi = simulateYAtX(R, dz, v0, hi, k);
    if (yLo == null || yHi == null) return new Shot(0, 0, 0, 0, false);

    // f(theta) = y_at_R - dz; root is zero
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
      return new Shot(bestTheta, 0, bestT, 0, bestAbs < 0.10); // 10 cm tolerance
    }

    // Bisection
    double mid = 0;
    SimResult midRes = null;

    for (int iter = 0; iter < 30; iter++) {
      mid = 0.5 * (lo + hi);
      SimResult sr = simulateToX(R, v0, mid, k);
      if (!sr.ok) return new Shot(0, 0, 0, 0, false);

      double fMid = sr.y - dz;
      midRes = sr;

      if (Math.abs(fMid) < 0.01) { // 1 cm
        return new Shot(mid, 0, sr.t, 0, true);
      }

      if (Math.signum(fMid) == Math.signum(fLo)) {
        lo = mid;
        fLo = fMid;
      } else {
        hi = mid;
        fHi = fMid;
      }
    }

    if (midRes == null) return new Shot(0, 0, 0, 0, false);
    return new Shot(mid, 0, midRes.t, 0, true);
  }

  private static class SimResult {
    final double y;
    final double t;
    final boolean ok;
    SimResult(double y, double t, boolean ok) { this.y = y; this.t = t; this.ok = ok; }
  }

  /**
   * Returns f(theta) = y_at_R - dz (root = 0), or null if simulation failed.
   */
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

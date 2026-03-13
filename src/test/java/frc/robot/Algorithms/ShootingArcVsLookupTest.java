package frc.robot.Algorithms;

/**
 * Standalone comparison: ShootingArc physics-based algorithm vs empirical lookup table.
 *
 * No external dependencies — all physics is reimplemented here.
 *
 * Gear ratio: 5 flywheel rotations per 3 motor rotations (18:30 tooth = 5:3 step-up)
 * Flywheel diameter: 4 inches = 0.1016 m
 * Conversion: exitSpeedMps = |motorRPS| * (5/3) * pi * 0.1016
 *
 * Hood: linear map  rotations [0.05, 1.70] <-> degrees [22.0, 65.0]
 *
 * Run:  javac src/test/java/frc/robot/Algorithms/ShootingArcVsLookupTest.java
 *       java -cp src/test/java frc.robot.Algorithms.ShootingArcVsLookupTest
 */
public class ShootingArcVsLookupTest {

    // ── Robot / field constants (from Constants.java) ──────────────────────
    static final double G              = 9.80665;
    static final double SHOOTER_H_M    = 21.5 * 0.0254;         // 0.5461 m
    static final double GOAL_H_M       = 72.0 * 0.0254;         // 1.8288 m
    static final double DZ             = GOAL_H_M - SHOOTER_H_M; // 1.2827 m

    // ── Launcher constants ─────────────────────────────────────────────────
    static final double HOOD_MIN_ROT   = 0.05;
    static final double HOOD_MAX_ROT   = 1.70;
    static final double HOOD_MIN_DEG   = 22.0;
    static final double HOOD_MAX_DEG   = 65.0;
    static final double HOOD_MIN_RAD   = Math.toRadians(HOOD_MIN_DEG);
    static final double HOOD_MAX_RAD   = Math.toRadians(HOOD_MAX_DEG);
    static final double WHEEL_DIAM_M   = 0.1016;
    static final double GEAR_RATIO     = 5.0 / 3.0;             // flywheel:motor
    static final double DRAG_K         = 0.03;
    static final double MAX_EXIT_MPS   = 25.0;

    // ── Lookup table data (from LaunchLookup.java, hub tables) ────────────
    static final double[][] HOOD_LUT  = {{ 1.5, 0.05 }, { 2.09, 0.05 }, { 3.15, 0.05 }, { 4.15, 0.94 }};
    static final double[][] SPEED_LUT = {{ 1.5, 50.0 }, { 2.09, 55.0 }, { 3.15, 75.0 }, { 4.25, 70.0 }};

    // ── Unit conversions ──────────────────────────────────────────────────

    static double rotToDeg(double rot) {
        return HOOD_MIN_DEG + (rot - HOOD_MIN_ROT) / (HOOD_MAX_ROT - HOOD_MIN_ROT) * (HOOD_MAX_DEG - HOOD_MIN_DEG);
    }

    static double degToRot(double deg) {
        return HOOD_MIN_ROT + (deg - HOOD_MIN_DEG) / (HOOD_MAX_DEG - HOOD_MIN_DEG) * (HOOD_MAX_ROT - HOOD_MIN_ROT);
    }

    /** Motor RPS -> ball exit speed (m/s). */
    static double rpsToMps(double motorRps) {
        return Math.abs(motorRps) * GEAR_RATIO * Math.PI * WHEEL_DIAM_M;
    }

    /** Ball exit speed (m/s) -> motor RPS. */
    static double mpsToRps(double mps) {
        return mps / (GEAR_RATIO * Math.PI * WHEEL_DIAM_M);
    }

    // ── Piecewise linear interpolation (mimics InterpolatingDoubleTreeMap) ─

    static double lerp(double[][] table, double x) {
        if (x <= table[0][0]) return table[0][1];
        if (x >= table[table.length - 1][0]) return table[table.length - 1][1];
        for (int i = 0; i < table.length - 1; i++) {
            if (x <= table[i + 1][0]) {
                double t = (x - table[i][0]) / (table[i + 1][0] - table[i][0]);
                return table[i][1] + t * (table[i + 1][1] - table[i][1]);
            }
        }
        return table[table.length - 1][1];
    }

    // ── Physics: 2D point-mass simulation with quadratic drag ─────────────

    /** Integrate forward until horizontal distance >= R. Returns {y, t} or null. */
    static double[] simulateToX(double R, double v0, double theta, double k) {
        double dt = 0.005;
        double tMax = 3.0;
        double x = 0, y = 0;
        double vx = v0 * Math.cos(theta);
        double vy = v0 * Math.sin(theta);
        double t = 0;
        if (vx <= 0.1) return null;
        while (t < tMax) {
            if (x >= R) return new double[]{ y, t };
            if (y < -0.2) return null;
            double speed = Math.hypot(vx, vy);
            double ax = -k * speed * vx;
            double ay = -G - k * speed * vy;
            vx += ax * dt;
            vy += ay * dt;
            x  += vx * dt;
            y  += vy * dt;
            t  += dt;
        }
        return null;
    }

    /** Solve pitch with quadratic drag via bisection (mirrors ShootingArc.solvePitchWithQuadraticDrag). */
    static double[] solvePitchDrag(double R, double dz, double v0, double k, boolean highArc) {
        double lo = HOOD_MIN_RAD;
        double hi = HOOD_MAX_RAD;
        if (highArc) lo = Math.max(lo, Math.toRadians(25));

        double[] srLo = simulateToX(R, v0, lo, k);
        double[] srHi = simulateToX(R, v0, hi, k);
        if (srLo == null || srHi == null) return null;

        double fLo = srLo[0] - dz;
        double fHi = srHi[0] - dz;

        if (Math.signum(fLo) == Math.signum(fHi)) {
            double bestTheta = lo, bestAbs = Math.abs(fLo), bestT = 0;
            for (int i = 0; i <= 60; i++) {
                double th = lo + (hi - lo) * i / 60.0;
                double[] sr = simulateToX(R, v0, th, k);
                if (sr == null) continue;
                double f = sr[0] - dz;
                if (Math.abs(f) < bestAbs) { bestAbs = Math.abs(f); bestTheta = th; bestT = sr[1]; }
            }
            if (bestAbs < 0.10) return new double[]{ bestTheta, bestT };
            return null;
        }

        double mid = 0, midT = 0;
        for (int iter = 0; iter < 30; iter++) {
            mid = 0.5 * (lo + hi);
            double[] sr = simulateToX(R, v0, mid, k);
            if (sr == null) return null;
            double fMid = sr[0] - dz;
            midT = sr[1];
            if (Math.abs(fMid) < 0.01) return new double[]{ mid, midT };
            if (Math.signum(fMid) == Math.signum(fLo)) { lo = mid; fLo = fMid; }
            else { hi = mid; fHi = fMid; }
        }
        return new double[]{ mid, midT };
    }

    /** Auto-solve minimum viable exit speed (mirrors ShootingArc.solveSpeedAndPitchWithDrag). */
    static double[] solveAutoSpeed(double R, double dz, double k, boolean highArc) {
        double speedLo = 1.0, speedHi = MAX_EXIT_MPS;
        double[] best = solvePitchDrag(R, dz, speedHi, k, highArc);
        if (best == null) return null;

        for (int i = 0; i < 20; i++) {
            double mid = 0.5 * (speedLo + speedHi);
            double[] trial = solvePitchDrag(R, dz, mid, k, highArc);
            if (trial != null) { best = trial; speedHi = mid; }
            else { speedLo = mid; }
        }
        double solvedSpeed = Math.min(0.5 * (speedLo + speedHi), MAX_EXIT_MPS);
        return new double[]{ best[0], best[1], solvedSpeed };
    }

    /** No-drag ballistic: low-arc pitch solution. */
    static Double solvePitchNoDrag(double R, double dz, double v) {
        double v2 = v * v;
        double disc = v2 * v2 - G * (G * R * R + 2.0 * dz * v2);
        if (disc < 0) return null;
        double tanLow = (v2 - Math.sqrt(disc)) / (G * R);
        double angle = Math.atan(tanLow);
        if (angle < HOOD_MIN_RAD || angle > HOOD_MAX_RAD) return null;
        return angle;
    }

    /**
     * Solve for the required exit speed at a FIXED pitch angle with drag.
     * Binary search: find the speed where the ball lands at height dz at range R.
     * Returns {exitSpeedMps, flightTimeS} or null.
     */
    static double[] solveSpeedAtFixedAngle(double R, double dz, double thetaRad, double k) {
        double speedLo = 1.0, speedHi = 80.0; // wide search range
        // Check if max speed can even overshoot
        double[] srHi = simulateToX(R, speedHi, thetaRad, k);
        if (srHi == null) return null; // can't even reach R at max speed

        // We want f(speed) = simY - dz = 0
        // At low speed the ball falls short (y < dz), at high speed it overshoots (y > dz)
        double[] srLo = simulateToX(R, speedLo, thetaRad, k);
        double fHi = srHi[0] - dz;

        // If even at 80 m/s the ball is below target, no solution
        if (fHi < -0.01) return null;

        for (int i = 0; i < 30; i++) {
            double mid = 0.5 * (speedLo + speedHi);
            double[] sr = simulateToX(R, mid, thetaRad, k);
            if (sr == null) {
                // Ball didn't reach R — need more speed
                speedLo = mid;
                continue;
            }
            double fMid = sr[0] - dz;
            if (Math.abs(fMid) < 0.005) return new double[]{ mid, sr[1] }; // 5mm tolerance
            if (fMid < 0) {
                speedLo = mid; // too slow, ball drops below target
            } else {
                speedHi = mid; // too fast, ball is above target
            }
        }
        double finalSpeed = 0.5 * (speedLo + speedHi);
        double[] sr = simulateToX(R, finalSpeed, thetaRad, k);
        if (sr != null && Math.abs(sr[0] - dz) < 0.10) return new double[]{ finalSpeed, sr[1] };
        return null;
    }

    // ═══════════════════════════════════════════════════════════════════════
    public static void main(String[] args) {

        System.out.println();
        System.out.println("=".repeat(105));
        System.out.println("  ShootingArc (physics) vs Lookup Table (empirical) Comparison");
        System.out.println("=".repeat(105));
        System.out.printf("  dz = %.4f m  |  drag k = %.2f  |  wheel dia = %.4f m  |  gear = 5:3 step-up%n",
                DZ, DRAG_K, WHEEL_DIAM_M);
        System.out.printf("  Motor RPS -> m/s formula:  |RPS| * (5/3) * pi * 0.1016%n");
        System.out.printf("  Example: 50 RPS -> %.2f m/s   75 RPS -> %.2f m/s%n", rpsToMps(50), rpsToMps(75));
        System.out.println();

        double[] distances = { 1.50, 2.09, 3.15, 4.15 };

        // ──────────────────────────────────────────────────────────────────
        //  PART 1: Same exit speed — compare hood angle only
        // ──────────────────────────────────────────────────────────────────
        System.out.println("  PART 1: Same exit speed as lookup table — compare hood angle only");
        System.out.println("  (ShootingArc solves pitch with quadratic drag for the LUT's exit speed)");
        System.out.println();
        System.out.printf("  %-7s | %-34s | %-22s | %s%n",
                "Dist", "LOOKUP TABLE", "SHOOTINGARC (drag)", "DELTA");
        System.out.printf("  %-7s | %7s %7s %8s | %7s %7s %7s | %s%n",
                "", "HoodDeg", "MotRPS", "ExitM/s", "HoodDeg", "HoodRot", "Flight", "dHood");
        System.out.println("  " + "-".repeat(95));

        for (double d : distances) {
            double lutRot = lerp(HOOD_LUT, d);
            double lutRps = lerp(SPEED_LUT, d);
            double lutDeg = rotToDeg(lutRot);
            double lutMps = rpsToMps(lutRps);

            double[] result = solvePitchDrag(d, DZ, lutMps, DRAG_K, false);

            if (result != null) {
                double arcDeg = Math.toDegrees(result[0]);
                double arcRot = degToRot(arcDeg);
                System.out.printf("  %5.2f m | %6.1f  %6.1f  %7.2f  | %6.1f  %6.3f  %5.3fs | %+5.1f deg%n",
                        d, lutDeg, lutRps, lutMps, arcDeg, arcRot, result[1], arcDeg - lutDeg);
            } else {
                System.out.printf("  %5.2f m | %6.1f  %6.1f  %7.2f  | [NO SOLUTION]          |%n",
                        d, lutDeg, lutRps, lutMps);
            }
        }

        // ──────────────────────────────────────────────────────────────────
        //  PART 2: Auto-speed — ShootingArc finds minimum exit speed
        // ──────────────────────────────────────────────────────────────────
        System.out.println();
        System.out.println("  " + "-".repeat(95));
        System.out.println();
        System.out.println("  PART 2: Auto-speed — ShootingArc finds MINIMUM viable exit speed with drag");
        System.out.println();
        System.out.printf("  %-7s | %22s | %30s | %s%n",
                "Dist", "LOOKUP TABLE", "SHOOTINGARC (auto-speed)", "DELTA");
        System.out.printf("  %-7s | %7s %7s %6s | %7s %7s %6s %7s | %8s %7s%n",
                "", "HoodDeg", "MotRPS", "m/s", "HoodDeg", "MotRPS", "m/s", "Flight", "dHood", "dRPS");
        System.out.println("  " + "-".repeat(95));

        for (double d : distances) {
            double lutRot = lerp(HOOD_LUT, d);
            double lutRps = lerp(SPEED_LUT, d);
            double lutDeg = rotToDeg(lutRot);
            double lutMps = rpsToMps(lutRps);

            double[] auto = solveAutoSpeed(d, DZ, DRAG_K, false);

            if (auto != null) {
                double autoDeg = Math.toDegrees(auto[0]);
                double autoMps = auto[2];
                double autoRps = mpsToRps(autoMps);
                System.out.printf("  %5.2f m | %6.1f  %6.1f  %5.2f | %6.1f  %6.1f  %5.2f  %5.3fs | %+6.1f  %+6.1f%n",
                        d, lutDeg, lutRps, lutMps,
                        autoDeg, autoRps, autoMps, auto[1],
                        autoDeg - lutDeg, autoRps - lutRps);
            } else {
                System.out.printf("  %5.2f m | %6.1f  %6.1f  %5.2f | [NO SOLUTION]                  |%n",
                        d, lutDeg, lutRps, lutMps);
            }
        }

        // ──────────────────────────────────────────────────────────────────
        //  PART 3: No-drag ballistic (analytical) for reference
        // ──────────────────────────────────────────────────────────────────
        System.out.println();
        System.out.println("  " + "-".repeat(95));
        System.out.println();
        System.out.println("  PART 3: No-drag ballistic (analytical) — same exit speed as LUT");
        System.out.println();
        System.out.printf("  %-7s | %8s | %9s | %11s | %8s%n",
                "Dist", "Exit m/s", "LUT Hood", "NoDrag Hood", "dHood");
        System.out.println("  " + "-".repeat(60));

        for (double d : distances) {
            double lutRps = lerp(SPEED_LUT, d);
            double lutDeg = rotToDeg(lerp(HOOD_LUT, d));
            double lutMps = rpsToMps(lutRps);

            Double noDragRad = solvePitchNoDrag(d, DZ, lutMps);
            if (noDragRad != null) {
                double noDragDeg = Math.toDegrees(noDragRad);
                System.out.printf("  %5.2f m | %7.2f  | %7.1f   | %7.1f     | %+6.1f deg%n",
                        d, lutMps, lutDeg, noDragDeg, noDragDeg - lutDeg);
            } else {
                System.out.printf("  %5.2f m | %7.2f  | %7.1f   | [UNREACH]   |%n",
                        d, lutMps, lutDeg);
            }
        }

        // ──────────────────────────────────────────────────────────────────
        //  PART 4: Fine-grained sweep every 0.25 m
        // ──────────────────────────────────────────────────────────────────
        System.out.println();
        System.out.println("  " + "-".repeat(95));
        System.out.println();
        System.out.println("  PART 4: Fine-grained sweep (0.25 m steps) — Drag vs NoDrag vs LUT");
        System.out.println("  Uses interpolated LUT exit speed at each distance");
        System.out.println();
        System.out.printf("  %-7s | %7s | %8s | %9s | %9s | %8s | %6s%n",
                "Dist", "Exit m/s", "LUT Hood", "Drag Hood", "NoDrag", "dDrag", "Flight");
        System.out.println("  " + "-".repeat(80));

        for (double d = 1.50; d <= 4.30; d += 0.25) {
            double lutRps = lerp(SPEED_LUT, d);
            double lutDeg = rotToDeg(lerp(HOOD_LUT, d));
            double lutMps = rpsToMps(lutRps);

            double[] dragResult = solvePitchDrag(d, DZ, lutMps, DRAG_K, false);
            Double noDragRad   = solvePitchNoDrag(d, DZ, lutMps);

            String dragStr   = dragResult != null ? String.format("%7.1f  ", Math.toDegrees(dragResult[0])) : " NO SOL  ";
            String noDragStr = noDragRad  != null ? String.format("%7.1f  ", Math.toDegrees(noDragRad))     : " NO SOL  ";
            String deltaStr  = dragResult != null ? String.format("%+6.1f  ", Math.toDegrees(dragResult[0]) - lutDeg) : "   --   ";
            String timeStr   = dragResult != null ? String.format("%5.3fs", dragResult[1]) : "  --  ";

            System.out.printf("  %5.2f m | %7.2f | %6.1f   | %s | %s | %s | %s%n",
                    d, lutMps, lutDeg, dragStr, noDragStr, deltaStr, timeStr);
        }

        // ──────────────────────────────────────────────────────────────────
        //  PART 5: MOTOR SPEED COMPARISON
        //  Fix the hood angle to the LUT value, solve for required speed
        // ──────────────────────────────────────────────────────────────────
        System.out.println();
        System.out.println("  " + "-".repeat(95));
        System.out.println();
        System.out.println("  PART 5: Motor speed comparison — at the LUT's hood angle, what speed does physics need?");
        System.out.println("  (Fixes hood to LUT value, solves for required exit speed with drag)");
        System.out.println();
        System.out.printf("  %-7s | %8s | %16s | %16s | %16s%n",
                "Dist", "Hood Deg", "LUT Motor", "Physics Motor", "DELTA");
        System.out.printf("  %-7s | %8s | %7s %7s | %7s %7s | %7s %7s%n",
                "", "", "RPS", "m/s", "RPS", "m/s", "dRPS", "dm/s");
        System.out.println("  " + "-".repeat(95));

        for (double d : distances) {
            double lutRot = lerp(HOOD_LUT, d);
            double lutRps = lerp(SPEED_LUT, d);
            double lutDeg = rotToDeg(lutRot);
            double lutMps = rpsToMps(lutRps);
            double lutThetaRad = Math.toRadians(lutDeg);

            double[] speedResult = solveSpeedAtFixedAngle(d, DZ, lutThetaRad, DRAG_K);

            if (speedResult != null) {
                double physMps = speedResult[0];
                double physRps = mpsToRps(physMps);
                System.out.printf("  %5.2f m | %6.1f   | %6.1f  %6.2f  | %6.1f  %6.2f  | %+6.1f  %+6.2f%n",
                        d, lutDeg, lutRps, lutMps, physRps, physMps,
                        physRps - lutRps, physMps - lutMps);
            } else {
                System.out.printf("  %5.2f m | %6.1f   | %6.1f  %6.2f  | [NO SOLUTION]   |%n",
                        d, lutDeg, lutRps, lutMps);
            }
        }

        // ── Part 5b: Fine-grained speed sweep ──
        System.out.println();
        System.out.println("  Fine-grained speed sweep (0.25 m steps):");
        System.out.println();
        System.out.printf("  %-7s | %7s | %7s | %7s | %7s | %7s%n",
                "Dist", "HoodDeg", "LUT RPS", "Phys RPS", "dRPS", "Flight");
        System.out.println("  " + "-".repeat(65));

        for (double d = 1.50; d <= 4.30; d += 0.25) {
            double lutRot = lerp(HOOD_LUT, d);
            double lutRps = lerp(SPEED_LUT, d);
            double lutDeg = rotToDeg(lutRot);
            double lutThetaRad = Math.toRadians(lutDeg);

            double[] speedResult = solveSpeedAtFixedAngle(d, DZ, lutThetaRad, DRAG_K);

            if (speedResult != null) {
                double physRps = mpsToRps(speedResult[0]);
                System.out.printf("  %5.2f m | %5.1f   | %6.1f  | %6.1f   | %+6.1f  | %5.3fs%n",
                        d, lutDeg, lutRps, physRps, physRps - lutRps, speedResult[1]);
            } else {
                System.out.printf("  %5.2f m | %5.1f   | %6.1f  | NO SOL  |    --   |   -- %n",
                        d, lutDeg, lutRps);
            }
        }

        System.out.println();
        System.out.println("=".repeat(105));
        System.out.println("  + dHood means ShootingArc says aim HIGHER than lookup table");
        System.out.println("  - dHood means ShootingArc says aim LOWER than lookup table");
        System.out.println("  + dRPS  means physics needs MORE motor speed than lookup table");
        System.out.println("  - dRPS  means physics needs LESS motor speed than lookup table");
        System.out.println("=".repeat(105));
        System.out.println();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretCalibration;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Calibrates the turret by finding both limit switches, then waiting for the
 * operator to hand-rotate the turret to the true zero position.
 *
 * <p><b>State machine:</b>
 * <ol>
 *   <li>SEEK_NEG – slowly spin CW (negative) until the neg limit switch trips.
 *       Record the motor position.</li>
 *   <li>BACK_OFF_NEG – back off the neg switch so we can keep spinning.</li>
 *   <li>SEEK_POS – continue spinning CCW (positive) until the pos limit switch trips.
 *       Record the motor position.</li>
 *   <li>BACK_OFF_POS – back off the pos switch.</li>
 *   <li>WAIT_FOR_ZERO – stop the motor. Print estimated switch positions.
 *       The operator physically rotates the turret to the true-zero position and
 *       presses a button on SmartDashboard ("TurretCalibration/SetZero") to confirm.</li>
 *   <li>DONE – calculate and save calibration, then finish.</li>
 * </ol>
 *
 * <p><b>Geometry:</b> The two switches are 4-1/8″ from turret center, 1″ apart
 * → angular separation ≈ 13.93°.
 */
public class CalibrateTurret extends Command {

    private enum State {
        SEEK_NEG,
        BACK_OFF_NEG,
        SEEK_POS,
        BACK_OFF_POS,
        WAIT_FOR_ZERO,
        DONE
    }

    private static final double SEEK_SPEED = 0.08;   // slow crawl
    private static final double BACK_OFF_SPEED = 0.08;
    private static final int BACK_OFF_TICKS = 25;     // ~0.5 s at 50 Hz

    private final TurretSubsystem turret;
    private final TurretCalibration calibration;

    private State state;
    private double negSwitchRawRot;
    private double posSwitchRawRot;
    private int backOffCounter;

    public CalibrateTurret(TurretSubsystem turret, TurretCalibration calibration) {
        this.turret = turret;
        this.calibration = calibration;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        state = State.SEEK_NEG;
        negSwitchRawRot = Double.NaN;
        posSwitchRawRot = Double.NaN;
        backOffCounter = 0;

        // Put a button on the dashboard for the user to confirm zero
        SmartDashboard.putBoolean("TurretCalibration/SetZero", false);
        SmartDashboard.putString("TurretCalibration/Phase", "SEEK_NEG – Searching for negative switch...");

        System.out.println("[CalibrateTurret] Starting calibration – seeking negative switch...");
    }

    @Override
    public void execute() {
        switch (state) {

            case SEEK_NEG:
                turret.setSpeed(-SEEK_SPEED);
                if (turret.isAtNegativeLimit()) {
                    negSwitchRawRot = turret.getRawMotorRotations();
                    turret.stop();
                    state = State.BACK_OFF_NEG;
                    backOffCounter = 0;
                    SmartDashboard.putString("TurretCalibration/Phase",
                            "BACK_OFF_NEG – Found neg switch at " +
                            String.format("%.3f", negSwitchRawRot) + " rot. Backing off...");
                    System.out.println("[CalibrateTurret] Neg switch at motor rot = " + negSwitchRawRot);
                }
                break;

            case BACK_OFF_NEG:
                turret.setSpeed(BACK_OFF_SPEED);
                backOffCounter++;
                if (backOffCounter >= BACK_OFF_TICKS && !turret.isAtNegativeLimit()) {
                    turret.stop();
                    state = State.SEEK_POS;
                    SmartDashboard.putString("TurretCalibration/Phase",
                            "SEEK_POS – Searching for positive switch...");
                    System.out.println("[CalibrateTurret] Backed off neg, seeking pos switch...");
                }
                break;

            case SEEK_POS:
                turret.setSpeed(BACK_OFF_SPEED);
                if (turret.isAtPositiveLimit()) {
                    posSwitchRawRot = turret.getRawMotorRotations();
                    turret.stop();
                    state = State.BACK_OFF_POS;
                    backOffCounter = 0;
                    SmartDashboard.putString("TurretCalibration/Phase",
                            "BACK_OFF_POS – Found pos switch at " +
                            String.format("%.3f", posSwitchRawRot) + " rot. Backing off...");
                    System.out.println("[CalibrateTurret] Pos switch at motor rot = " + posSwitchRawRot);
                }
                break;

            case BACK_OFF_POS:
                turret.setSpeed(-BACK_OFF_SPEED);
                backOffCounter++;
                if (backOffCounter >= BACK_OFF_TICKS && !turret.isAtPositiveLimit()) {
                    turret.stop();
                    state = State.WAIT_FOR_ZERO;
                    printSwitchEstimate();
                    SmartDashboard.putString("TurretCalibration/Phase",
                            "WAIT_FOR_ZERO – Rotate turret to TRUE ZERO by hand, then set " +
                            "'TurretCalibration/SetZero' = true");
                    System.out.println("[CalibrateTurret] Rotate turret to true zero by hand.");
                    System.out.println("[CalibrateTurret] Then set SmartDashboard 'TurretCalibration/SetZero' = true.");
                }
                break;

            case WAIT_FOR_ZERO:
                // Motor is stopped; user physically rotates turret
                // Continuously show current motor position so user can see movement
                double currentRot = turret.getRawMotorRotations();
                double currentDeg = TurretCalibration.motorRotToDeg(currentRot);
                SmartDashboard.putNumber("TurretCalibration/CurrentMotorRot", currentRot);
                SmartDashboard.putNumber("TurretCalibration/CurrentDeg (uncalibrated)", currentDeg);

                if (SmartDashboard.getBoolean("TurretCalibration/SetZero", false)) {
                    double zeroRawRot = turret.getRawMotorRotations();
                    calibration.calibrate(negSwitchRawRot, posSwitchRawRot, zeroRawRot);

                    System.out.println("[CalibrateTurret] Zero set at motor rot = " + zeroRawRot);
                    System.out.println("[CalibrateTurret] Neg switch angle = " +
                            String.format("%.2f", calibration.getNegSwitchDeg()) + "°");
                    System.out.println("[CalibrateTurret] Pos switch angle = " +
                            String.format("%.2f", calibration.getPosSwitchDeg()) + "°");
                    System.out.println("[CalibrateTurret] Calibration saved to disk!");

                    SmartDashboard.putString("TurretCalibration/Phase", "DONE – Calibration complete!");
                    SmartDashboard.putNumber("TurretCalibration/NegSwitchAngleDeg", calibration.getNegSwitchDeg());
                    SmartDashboard.putNumber("TurretCalibration/PosSwitchAngleDeg", calibration.getPosSwitchDeg());

                    // Apply calibration to the turret immediately
                    turret.applyCalibration();

                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    private void printSwitchEstimate() {
        double spanRot = posSwitchRawRot - negSwitchRawRot;
        double spanDeg = TurretCalibration.motorRotToDeg(spanRot);

        System.out.println("=== SWITCH ESTIMATE (before zero set) ===");
        System.out.println("  Neg switch raw motor rot: " + String.format("%.3f", negSwitchRawRot));
        System.out.println("  Pos switch raw motor rot: " + String.format("%.3f", posSwitchRawRot));
        System.out.println("  Span between switches:    " + String.format("%.2f", spanDeg) + "°");
        System.out.println("  Expected separation:      " + String.format("%.2f", TurretCalibration.SWITCH_SEPARATION_DEG) + "°");
        System.out.println("==========================================");

        SmartDashboard.putNumber("TurretCalibration/SwitchSpanDeg", spanDeg);
        SmartDashboard.putNumber("TurretCalibration/ExpectedSeparationDeg", TurretCalibration.SWITCH_SEPARATION_DEG);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        SmartDashboard.putBoolean("TurretCalibration/SetZero", false);
        if (interrupted) {
            SmartDashboard.putString("TurretCalibration/Phase", "INTERRUPTED");
            System.out.println("[CalibrateTurret] Calibration interrupted!");
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }
}

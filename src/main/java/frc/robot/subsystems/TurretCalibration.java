package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Persists turret switch calibration data to a file on the RoboRIO so the
 * turret can recover its absolute angle after a power cycle.
 *
 * File location: /home/lvuser/turret_calibration.txt
 *
 * Stored values (all in motor rotations, relative to true-zero):
 *   negSwitchMotorRot  – motor position when the negative limit switch fires
 *   posSwitchMotorRot  – motor position when the positive limit switch fires
 */
public class TurretCalibration {

    private static final String FILE_PATH = "/home/lvuser/turret_calibration.txt";

    private double negSwitchMotorRot = Double.NaN;
    private double posSwitchMotorRot = Double.NaN;
    private boolean loaded = false;

    // Geometry constants
    private static final double SWITCH_RADIUS_INCHES = 4.125; // 4 + 1/8 inches
    private static final double SWITCH_SPACING_INCHES = 1.0;
    private static final double GEAR_RATIO = 5000.0 / 120.0; // ~41.667

    /** Angular separation between the two switches (degrees). */
    public static final double SWITCH_SEPARATION_DEG =
            2.0 * Math.toDegrees(Math.asin(SWITCH_SPACING_INCHES / (2.0 * SWITCH_RADIUS_INCHES)));

    public TurretCalibration() {
        load();
    }

    /** Try to load calibration from disk. */
    public void load() {
        File f = new File(FILE_PATH);
        if (!f.exists()) {
            loaded = false;
            return;
        }
        try (BufferedReader br = new BufferedReader(new FileReader(f))) {
            negSwitchMotorRot = Double.parseDouble(br.readLine().trim());
            posSwitchMotorRot = Double.parseDouble(br.readLine().trim());
            loaded = true;
            SmartDashboard.putString("TurretCalibration/Status", "Loaded from disk");
            SmartDashboard.putNumber("TurretCalibration/NegSwitchMotorRot", negSwitchMotorRot);
            SmartDashboard.putNumber("TurretCalibration/PosSwitchMotorRot", posSwitchMotorRot);
            SmartDashboard.putNumber("TurretCalibration/NegSwitchDeg", motorRotToDeg(negSwitchMotorRot));
            SmartDashboard.putNumber("TurretCalibration/PosSwitchDeg", motorRotToDeg(posSwitchMotorRot));
        } catch (IOException | NumberFormatException e) {
            loaded = false;
            SmartDashboard.putString("TurretCalibration/Status", "Load FAILED: " + e.getMessage());
        }
    }

    /** Save current calibration to disk. */
    public void save() {
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(FILE_PATH))) {
            bw.write(Double.toString(negSwitchMotorRot));
            bw.newLine();
            bw.write(Double.toString(posSwitchMotorRot));
            bw.newLine();
            SmartDashboard.putString("TurretCalibration/Status", "Saved to disk");
        } catch (IOException e) {
            SmartDashboard.putString("TurretCalibration/Status", "Save FAILED: " + e.getMessage());
        }
    }

    public boolean isCalibrated() {
        return loaded && !Double.isNaN(negSwitchMotorRot) && !Double.isNaN(posSwitchMotorRot);
    }

    public double getNegSwitchMotorRot() { return negSwitchMotorRot; }
    public double getPosSwitchMotorRot() { return posSwitchMotorRot; }

    public double getNegSwitchDeg() { return motorRotToDeg(negSwitchMotorRot); }
    public double getPosSwitchDeg() { return motorRotToDeg(posSwitchMotorRot); }

    public void setNegSwitchMotorRot(double rot) { this.negSwitchMotorRot = rot; }
    public void setPosSwitchMotorRot(double rot) { this.posSwitchMotorRot = rot; }

    /**
     * Store calibration from raw measurements.
     *
     * @param negSwitchRawRot motor rotation when neg switch fired (boot-relative)
     * @param posSwitchRawRot motor rotation when pos switch fired (boot-relative)
     * @param zeroRawRot      motor rotation when user declared true zero
     */
    public void calibrate(double negSwitchRawRot, double posSwitchRawRot, double zeroRawRot) {
        // Convert to zero-relative motor rotations
        this.negSwitchMotorRot = negSwitchRawRot - zeroRawRot;
        this.posSwitchMotorRot = posSwitchRawRot - zeroRawRot;
        this.loaded = true;
        save();

        SmartDashboard.putNumber("TurretCalibration/NegSwitchMotorRot", negSwitchMotorRot);
        SmartDashboard.putNumber("TurretCalibration/PosSwitchMotorRot", posSwitchMotorRot);
        SmartDashboard.putNumber("TurretCalibration/NegSwitchDeg", getNegSwitchDeg());
        SmartDashboard.putNumber("TurretCalibration/PosSwitchDeg", getPosSwitchDeg());
    }

    /** Convert motor rotations to turret degrees. */
    public static double motorRotToDeg(double motorRot) {
        return (motorRot / GEAR_RATIO) * 360.0;
    }

    /** Convert turret degrees to motor rotations. */
    public static double degToMotorRot(double deg) {
        return (deg / 360.0) * GEAR_RATIO;
    }
}

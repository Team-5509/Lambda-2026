package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.generated.TunerConstants;

public class TurretSubsystem extends SubsystemBase {

    /* ==================== Hardware IDs ==================== */
    private static final int TURRET_MOTOR_ID = Constants.TurretSubsystemConstants.kTurretMotorId;

    private static final int LIMIT_NEG_ID = 0; // -180 deg
    private static final int LIMIT_POS_ID = 1; // +180 deg

    /* ==================== Constants ==================== */
    // Turret rotations (1 rotation = 360 degrees)
    private static final double MIN_TURRET_ROT = -0.75;
    private static final double MAX_TURRET_ROT = 0.25;

    // private DoubleSupplier robotHeadingDegSupplier = () -> 0.0;

    // Motion Magic
    private static final double MM_CRUISE_VEL = 2.0; // rot/s
    private static final double MM_ACCEL = 6.0; // rot/s^2
    private static final double MM_JERK = 60.0; // rot/s^3

    /* ==================== Hardware ==================== */
    private final TalonFX turretMotor = new TalonFX(TURRET_MOTOR_ID);

    private final DigitalInput negLimit = new DigitalInput(LIMIT_NEG_ID);
    private final DigitalInput posLimit = new DigitalInput(LIMIT_POS_ID);

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /**
     * Constructs the TurretSubsystem. Configures the encoder and applies
     * Motion Magic PID, soft limits, and neutral mode to the turret motor.
     */
    public TurretSubsystem() {
        configureEncoder();
        configureMotor();
    }

    /* ==================== Configuration ==================== */

    private void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // CANcoder always reports ±0.5 rotations (±180°) in Phoenix 6
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        /* ---- Feedback ---- */
       
        /* ---- Motion Magic ---- */
        config.MotionMagic.MotionMagicCruiseVelocity = MM_CRUISE_VEL;
        config.MotionMagic.MotionMagicAcceleration = MM_ACCEL;
        config.MotionMagic.MotionMagicJerk = MM_JERK;

        /* ---- PID ---- */
        config.Slot0.kP = 60.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 5.0;
        config.Slot0.kV = 0.0;

        /* ---- Soft Limits ---- */
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_TURRET_ROT;
         
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_TURRET_ROT;

        /* ---- Motor ---- */
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        turretMotor.getConfigurator().apply(config);
    }

    /**
     * Command that sets the turret position with magic motion (closed loop
     * control).
     *
     * @return a command
     */
    public Command SetTurretPositionMM(DoubleSupplier positionSupplier) {
        return runOnce(() -> {
            turretMotor.setControl(
                    motionMagic.withPosition(positionSupplier.getAsDouble())
                            .withSlot(0));
        });
    }



    /**
     * Aims the turret at a target while compensating for robot and projectile travel time.
     * Predicts the robot's future position based on current velocity and flight time,
     * then aims at the field-relative angle to the predicted intercept point.
     *
     * @param robotPose       Current robot pose on the field
     * @param robotVelocity   Current field-relative robot velocity (m/s)
     * @param targetPosition  Field-relative target position (e.g., hub center)
     * @param projectileSpeed Projectile exit speed in m/s used to estimate flight time
     */
    public void aimFieldRelativeWithPrediction(
            Pose2d robotPose,
            Translation2d robotVelocity,
            Translation2d targetPosition,
            double projectileSpeed) {
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d toTarget = targetPosition.minus(robotPos);

        double distance = toTarget.getNorm();
        double flightTime = distance / projectileSpeed;

        Translation2d predictedRobotPos = robotPos.plus(robotVelocity.times(flightTime));

        Translation2d predictedVector = targetPosition.minus(predictedRobotPos);

        double angleDeg = Math.toDegrees(Math.atan2(
                predictedVector.getY(),
                predictedVector.getX()));

        aimFieldRelative(angleDeg);
    }

    /* ==================== Control ==================== */

    /**
     * Commands the turret to the specified angle (degrees), clamped to the allowed rotation range.
     * Converts degrees to motor rotations using the gear ratio and issues a Motion Magic position command.
     *
     * @param degrees Target turret angle in degrees (field-relative)
     */
    public void setTurretAngleDegrees(double degrees) {
        SmartDashboard.putNumber("TurretSubsystem/SetpointDegrees", degrees);
        degrees = Math.max(Constants.TurretSubsystemConstants.minTurretRotation, Math.min(Constants.TurretSubsystemConstants.maxTurretRotation, degrees));
        //TODO FIND THE ACTUAL MIN TURRET ROTATION
        if(degrees < (-90) ) {
            degrees = (Constants.TurretSubsystemConstants.maxTurretRotation + (Constants.TurretSubsystemConstants.maxTurretRotation - Math.abs(degrees)));
        }
        double rotations = degrees / 360.0;
        rotations = (rotations * Constants.TurretSubsystemConstants.gearRatio);
        SmartDashboard.putNumber("TurretSubsystem/SetpointRotations", rotations);
        turretMotor.setControl(
                motionMagic.withPosition(rotations));
    }

    /** Stops the turret motor immediately. */
    public void stop() {
        turretMotor.stopMotor();
    }

    /**
     * Sets the turret motor to the specified duty-cycle speed.
     *
     * @param speed Motor output speed (-1.0 to 1.0)
     */
      public void setSpeed(double speed) {
        turretMotor.set(speed);
      }

    /* ==================== State ==================== */

    /**
     * Returns the current turret position in degrees.
     *
     * @return Turret position in degrees (CCW+)
     */
    public double getTurretDegrees() {
        return turretMotor.getPosition().getValue().in(Units.Degrees);
    }

    /** Robot heading in field coordinates (CCW+, degrees, 0 = field forward) */
    // public void setRobotHeadingSupplier(DoubleSupplier supplier) {
    //     this.robotHeadingDegSupplier = supplier;
    // }

    /* ==================== Field-Oriented Control ==================== */

    /**
     * Aim turret at a field-relative angle
     * 
     * @param fieldAngleDeg CCW+, degrees
     */
    public void aimFieldRelative(double fieldAngleDeg) {
        double robotHeading = drivetrain.getState().Pose.getRotation().getDegrees();
        double turretAngle = fieldAngleDeg - robotHeading;
        SmartDashboard.putNumber("TurretSubsystem/FieldAngleDeg", turretAngle);
        SmartDashboard.putNumber("TurretSubsystem/RobotHeadingDeg", robotHeading);
        setTurretAngleDegrees(wrapDegrees(turretAngle));
    }

    private double wrapDegrees(double degrees) {
        while (degrees > 180)
            degrees -= 360;
        while (degrees < -180)
            degrees += 360;
        return degrees;
    }

    /**
     * Returns whether the turret has reached the negative (minimum angle) limit switch.
     *
     * @return true if the negative limit switch is triggered
     */
    public boolean isAtNegativeLimit() {
        return negLimit.get();
    }

    /**
     * Returns whether the turret has reached the positive (maximum angle) limit switch.
     *
     * @return true if the positive limit switch is triggered
     */
    public boolean isAtPositiveLimit() {
        return posLimit.get();
    } 


    /**
     * Called every scheduler cycle. Publishes turret position to SmartDashboard and
     * auto-zeros the motor encoder when a limit switch is triggered, stopping the motor
     * if it is moving in the direction of the triggered limit.
     */
    @Override
    public void periodic() {
        // Optional: auto-zero if home switch hit
        SmartDashboard.putNumber("TurretSubsystem/TurretPosition", turretMotor.getPosition().getValueAsDouble());
        if (isAtNegativeLimit()) {
            turretMotor.setPosition(Constants.TurretSubsystemConstants.minNegTurretMotorRot);
            if (turretMotor.getVelocity().getValueAsDouble() < 0) {
                turretMotor.stopMotor();
            }
        }
        else if (isAtPositiveLimit()) {
            turretMotor.setPosition(Constants.TurretSubsystemConstants.maxPosTurretMotorRot);
            if (turretMotor.getVelocity().getValueAsDouble() > 0) {
                turretMotor.stopMotor();
            }
        }
    }
}

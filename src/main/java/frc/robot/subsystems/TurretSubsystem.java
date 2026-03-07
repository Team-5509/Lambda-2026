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
    //private static final int TURRET_CANCODER_ID = Constants.TurretSubsystemConstants.kTurretEncoderId;

    private static final int LIMIT_NEG_ID = 0; // -180 deg
    private static final int LIMIT_POS_ID = 1; // +180 deg

    /* ==================== Constants ==================== */
    // Turret rotations (1 rotation = 360 degrees)
    private static final double MIN_TURRET_ROT = -0.75;
    private static final double MAX_TURRET_ROT = 0.25;

    // private DoubleSupplier robotHeadingDegSupplier = () -> 0.0;

    // Motion Magic
    // Make Motion Magic very slow on purpose
    private static final double MM_CRUISE_VEL = 0.25; // rot/s (was 2.0)
    private static final double MM_ACCEL = 0.5; // rot/s^2 (was 6.0)
    private static final double MM_JERK = 5.0; // rot/s^3 (was 60.0)

    // Cap open-loop speed to a small value to keep turret motion very slow
    private static final double MAX_OPEN_LOOP_SPEED = 0.15;

    /* ==================== Hardware ==================== */
    private final TalonFX turretMotor = new TalonFX(TURRET_MOTOR_ID);
    //private final CANcoder turretEncoder = new CANcoder(TURRET_CANCODER_ID);

    private final DigitalInput negLimit = new DigitalInput(LIMIT_NEG_ID);
    private final DigitalInput posLimit = new DigitalInput(LIMIT_POS_ID);

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public TurretSubsystem() {
        //configureEncoder();
        configureMotor();
    }

    /* ==================== Configuration ==================== */

    // private void configureEncoder() {
    //     CANcoderConfiguration config = new CANcoderConfiguration();

    //     // CANcoder always reports ±0.5 rotations (±180°) in Phoenix 6
    //     config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    //     turretEncoder.getConfigurator().apply(config);
    // }
    //}

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        /* ---- Feedback ---- */
        //config.Feedback.FeedbackRemoteSensorID = TURRET_CANCODER_ID;
        // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // config.Feedback.SensorToMechanismRatio = 10.0 / 100.0; // encoder → turret

       
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
            // Commented out to prevent turret movement during testing/analysis
            // turretMotor.setControl(
            //         motionMagic.withPosition(positionSupplier.getAsDouble())
            //                 .withSlot(0));
        });
    }



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
    // Commented out to prevent turret movement during testing/analysis
    // turretMotor.setControl(
    //         motionMagic.withPosition(rotations));
    }

    public void stop() {
                // turretMotor.stopMotor();
    }

      public void setSpeed(double speed) {
                // turretMotor.set(speed);
      }

    /* ==================== State ==================== */

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

    public boolean isAtNegativeLimit() {
        return negLimit.get();
    }

    public boolean isAtPositiveLimit() {
        return posLimit.get();
    } 


    @Override
    public void periodic() {
        // Optional: auto-zero if home switch hit
        SmartDashboard.putNumber("TurretSubsystem/TurretPosition", turretMotor.getPosition().getValueAsDouble());
        if (isAtNegativeLimit()) {
            // Commented out to prevent forcing turret position when limit switches hit
            // turretMotor.setPosition(Constants.TurretSubsystemConstants.minNegTurretMotorRot);
            if (turretMotor.getVelocity().getValueAsDouble() < 0) {
                // turretMotor.stopMotor();
            }
        }
        else if (isAtPositiveLimit()) {
            // Commented out to prevent forcing turret position when limit switches hit
            // turretMotor.setPosition(Constants.TurretSubsystemConstants.maxPosTurretMotorRot);
            if (turretMotor.getVelocity().getValueAsDouble() > 0) {
                // turretMotor.stopMotor();
            }
        }
    }
}

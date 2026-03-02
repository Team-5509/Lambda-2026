package frc.robot.Constants;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static class Vision {
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltWelded);
    }

    // Intake
    public static final class IntakeSubsystemConstants {
        public static final int kIntakeMotorId = 14;
        public static final int kDeployIntakeMotorId = 21;
        // CAN ID for the absolute encoder (CANcoder) used by the deploy intake
        public static final int kDeployIntakeEncoderId = 22;

    }

    // Conveyor
    public static final class ConveyorSubsystemConstants {
        public static final int kConveyorMotorId = 15;
    }

    // Kicker
    public static final class KickerSubsystemConstants {
        public static final int kKickerMotorId = 16;
    }

    // Turret
    public static final class TurretSubsystemConstants {
    public static final ArrayList<Translation2d> blueHomePoses = new ArrayList<>(Arrays.asList(
        new Translation2d(2, 3), // example field coords
        new Translation2d(2, 5)  // example field coords
    ));
    public static final ArrayList<Translation2d> redHomePoses = new ArrayList<>(Arrays.asList(
        new Translation2d(11, 3), // example field coords
        new Translation2d(11, 5)  // example field coords
    ));
    public static final Translation2d blueHubPose = new Translation2d(4.612, 4.021328); // example field coords
    public static final Translation2d blueHomePose = new Translation2d(2, 2); // example field coords
    public static final Translation2d redHubPose = new Translation2d(11.901424 ,4.021328); // example field coords
    public static final Translation2d redHomePose = new Translation2d(10, 2); // example field coords
    public static final double blueLineZone = 4.611;
    public static final double redLineZone = 11.901424;
    public static final double g = 9.80665; // m/s^2

    // Measure these on the robot:
    public static final double shooterHeightM = 21.5 * 0.0254; // TODO: Measure this 21.5 in -> m
    public static final double goalHeightM = 72.0 * 0.0254; // 72 in -> m
    public static final double dz = goalHeightM - shooterHeightM;

    // Height of the home/amp goal — measure on the actual robot/field.
    public static final double homeGoalHeightM = 24.0 * 0.0254; // TODO: Measure actual home goal height (inches)
    public static final double homeDz = homeGoalHeightM - shooterHeightM;

    //TODO: Measure this from shooter exit to robot center 
    public static final Translation2d shooterOffsetRobot = new Translation2d(0.32, 0.18);

        public static final int kTurretMotorId = 17;
        public static final int kTurretEncoderId = 24;
        public static final Double ballSpeed = 22.0;
    }

    // Launcher
    public static final class LauncherSubsystemConstants {
        public static final int kLauncherMotorId = 18;
        public static final int kHoodMotorId = 19;
        public static final int kHoodEncoderId = 23;
        public static final double trueZero = -0.186523;
        public static final double hoodToEncoderRatio = 0.125;
        public static final double kShootSpinUpSeconds = 0.2; // seconds to wait for launcher to spin up before feeding

        // Hood position limits (mechanism rotations) — must match soft limits in LauncherSubsystem
        public static final double kHoodMinRot = 0.05;
        public static final double kHoodMaxRot = 1.70;

        // Physical angle corresponding to the min/max hood rotation positions.
        // TODO: Measure both values on the actual robot.
        public static final double kHoodMinAngleDeg = 20.0;
        public static final double kHoodMaxAngleDeg = 70.0;

        // Maximum ball exit speed the launcher can achieve (m/s).
        // TODO: Measure actual max exit speed on the robot.
        public static final double kMaxExitSpeedMps = 25.0;

        // Flywheel diameter used to convert ball exit speed (m/s) → launcher RPS.
        // TODO: Measure actual flywheel diameter.
        public static final double kLauncherWheelDiameterM = 0.1016; // 4 in default
    }

    // Climber
    public static final class ClimberSubsystemConstants {
        public static final int kClimberMotorId = 20;
    }

}

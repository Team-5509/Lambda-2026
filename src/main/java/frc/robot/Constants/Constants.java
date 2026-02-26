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

    //TODO: Measure this from shooter exit to robot center 
    public static final Translation2d shooterOffsetRobot = new Translation2d(0.32, 0.18);

        public static final int kTurretMotorId = 17;
        public static final Double ballSpeed = 22.0;
    }

    // Launcher
    public static final class LauncherSubsystemConstants {
        public static final int kLauncherMotorId = 18;
        public static final int kHoodMotorId = 19;
        public static final int kHoodEncoderId = 23;
        public static final double trueZero = -0.186523;
        public static final double hoodToEncoderRatio = 0.125;
        
    }

    // Climber
    public static final class ClimberSubsystemConstants {
        public static final int kClimberMotorId = 20;
    }

}

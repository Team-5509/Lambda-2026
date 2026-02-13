package frc.robot.Constants;

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
        public static final Translation2d hubPose = new Translation2d(16.54, 5.55); // example field coords
        public static final Translation2d homePose = new Translation2d(16.54, 5.55); // example field coords
        public static final int kTurretMotorId = 17;
        public static final Double ballSpeed = 22.0;
    }

    // Launcher
    public static final class LauncherSubsystemConstants {
        public static final int kLauncherMotorId = 18;
        public static final int kHoodMotorId = 19;
    }

    // Climber
    public static final class ClimberSubsystemConstants {
        private static final int kClimberMotorId = 20;
    }

}

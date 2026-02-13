package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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



    public static class PoseTrust {
        // Scale < 1.0 means trust camera more. Scale > 1.0 means trust odometry more.
        public static final double kVisionStdDevScaleInsideBump = 0.45;
        public static final double kVisionStdDevScaleOutsideBump = 1.35;

        // Field-space bump rectangles in meters (xMin, xMax, yMin, yMax).
        // Update these values to match your field mapping.
        public static final BumpZone[] kBumpZones = {
                new BumpZone(3.85, 5.70, 0, 7), // Bump 1
                // new BumpZone(11.10, 13.20, 6.20, 7.40), // Bump 2
                // new BumpZone(3.60, 5.70, 0.60, 1.80), // Bump 3
                // new BumpZone(11.10, 13.20, 0.60, 1.80) // Bump 4
        };

        public static boolean isInAnyBump(Translation2d positionMeters) {
            for (var bumpZone : kBumpZones) {
                if (bumpZone.contains(positionMeters)) {
                    return true;
                }
            }
            return false;
        }

        public static record BumpZone(
                double minXMeters, double maxXMeters, double minYMeters, double maxYMeters) {
            public boolean contains(Translation2d point) {
                return point.getX() >= minXMeters
                        && point.getX() <= maxXMeters
                        && point.getY() >= minYMeters
                        && point.getY() <= maxYMeters;
            }
        }
    }


}

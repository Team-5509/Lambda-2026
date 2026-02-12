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
        public static final String kCameraName = "MAIN_CAM";
        // Cam mounted facing forward, half a meter forward of center, quarter of a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.25), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
     }

     
// Intake
public static final class IntakeSubsystemConstants{
    public static final int kIntakeMotorId = 14;
    public static final Translation2d hubPose = new Translation2d(16.54, 5.55); // example field coords
    public static final Translation2d homePose = new Translation2d(16.54, 5.55); // example field coords

}

    //Conveyor
    public static final class ConveyorSubsystemConstants{
    public static final int kConveyorMotorId = 15;
    }
                                                                                                                                                                                                 
    // Kicker
    public static final class KickerSubsystemConstants{
    public static final int kKickerMotorId = 16;
    }

    //Turret
    public static final class TurretSubsystemConstants{
        public static final Translation2d speakerPose = new Translation2d(16.54, 5.55); // example field coords
    public static final int kTurretMotorId = 17;
    public static final Double ballSpeed = 22.0;
    }
    //Launcher 
    public static final class LauncherSubsystemConstants{
    public static final int kLauncherMotorId = 18;
    public static final int kLauncher2MotorId = 19;
    }
    //Climber
    public static final class ClimberSubsystemConstants{
        private static final int kClimberMotorId = 20;
    }

     }
 }

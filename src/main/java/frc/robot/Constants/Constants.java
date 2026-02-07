package frc.robot.Constants;

 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Rotation3d;
 import edu.wpi.first.math.geometry.Transform3d;
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

     public static class Ids {

// Intake
    private static final int kIntakeMotorId = 14;

    //Conveyor
    private static final int kConveyorMotorId = 15;

    // Kicker
    private static final int kKickerMotorId = 16;

    //Turret
    private static final int kTurretMotorId = 17;

    //Launcher 
    private static final int kLauncherMotorId = 18;
    private static final int kLauncher2MotorId = 19;

    //Climber

     }
 }

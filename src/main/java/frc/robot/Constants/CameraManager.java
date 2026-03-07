package frc.robot.Constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class CameraManager {
    public enum CameraProperties {

        // Back Center Camera
        CAM_FL("Apple",
            // Cam mounted facing forward, half a meter forward of center, half a meter up from center. 
            new Transform3d(new Translation3d(0, -11.691, 10.607), new Rotation3d(0, Math.toRadians(10), Math.toRadians(-180))), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1)),
        
        // Back Left Up Camera
        CAM_FR("Basil", 
            new Transform3d(new Translation3d(9.566, -9.020, 10.501), new Rotation3d(0, 0, Math.toRadians(-25))), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1)),
        
        // Rear Left Swerve Camera
        CAM_RL("Dragonfruit", 
            new Transform3d(new Translation3d(10.772, -10.572, 5.771), new Rotation3d(0, Math.toRadians(35), Math.toRadians(115))), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1)),
        
        // Rear Right Swerve Camera
        CAM_RR("Banana", 
            new Transform3d(new Translation3d(-10.771, -10.529, 5.771), new Rotation3d(0, Math.toRadians(35), Math.toRadians(-115))), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1)),

            // Back Right Up Camera
        CAM_R("Camera", 
            new Transform3d(new Translation3d(-9.566, -9.020, 10.499), new Rotation3d(0, 0, Math.toRadians(25))), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1));

        


        public final String name;
        public final Transform3d transform;
        public final Matrix<N3, N1> singleTagStdDevs;
        public final Matrix<N3, N1> multiTagStdDevs;

        private CameraProperties(String name, 
                Transform3d transform, 
                Matrix<N3, N1> singleTagStdDevs, 
                Matrix<N3, N1> multiTagStdDevs) {
            this.name = name;
            this.transform = transform;
            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevs;
        }

        // Getters
        public String getName() {
            return name;
        }

        public Transform3d getTransform() {
            return transform;
        }

        public Matrix<N3, N1> getSingleTagStdDevs() {
            return singleTagStdDevs;
        }

        public Matrix<N3, N1> getMultiTagStdDevs() {
            return multiTagStdDevs;
        }
    }
}
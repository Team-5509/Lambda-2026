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
        // Front Left Camera
        CAM_FL("CAM_FL",
            // Cam mounted facing forward, half a meter forward of center, half a meter up from center. 
            new Transform3d(new Translation3d(0.276225, 0.276225, 0.19), new Rotation3d(0, Math.toRadians(-20), 45)), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1)),
        
        // Front Right Camera
        CAM_FR("CAM_FR", 
            new Transform3d(new Translation3d(0.276225, -0.276225, 0.19), new Rotation3d(0, Math.toRadians(-20), -45)), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1)),
        
        // Rear Left Camera
        CAM_RL("CAM_RL", 
            new Transform3d(new Translation3d(-0.276225, 0.276225, 0.19), new Rotation3d(0, Math.toRadians(20), 135)), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1)),
        
        // Rear Right Camera
        CAM_RR("CAM_RR", 
            new Transform3d(new Translation3d(-0.276225, -0.276225, 0.19), new Rotation3d(0, Math.toRadians(20), -135)), 
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
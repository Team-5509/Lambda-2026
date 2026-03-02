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
        
        CAM_FL("Apple",
            // Cam mounted facing forward, half a meter forward of center, half a meter up from center. 
            new Transform3d(new Translation3d(0.276225, 0.276225, 0.19), new Rotation3d(0, Math.toRadians(-20), Math.toRadians(45))), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1)),
        
        // Front Right Camera
        CAM_FR("Basil", 
            new Transform3d(new Translation3d(0.276225, -0.276225, 0.19), new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-45))), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1)),
        
        // Rear Left Camera
        CAM_RL("Dragonfruit", 
            new Transform3d(new Translation3d(-0.276225, 0.276225, 0.19), new Rotation3d(0, Math.toRadians(20), Math.toRadians(135))), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1)),
        
        // Rear Right Camera
        CAM_RR("Banana", 
            new Transform3d(new Translation3d(-0.276225, -0.276225, 0.19), new Rotation3d(0, Math.toRadians(20), Math.toRadians(-135))), 
            VecBuilder.fill(4, 4, 8), 
            VecBuilder.fill(0.5, 0.5, 1));

        public final String name;
        public final Transform3d transform;
        public final Matrix<N3, N1> singleTagStdDevs;
        public final Matrix<N3, N1> multiTagStdDevs;

        /**
         * Constructs a CameraProperties enum constant with camera configuration data.
         *
         * @param name            The PhotonVision camera name used to identify the camera
         * @param transform       The 3D transform from robot center to camera position/orientation
         * @param singleTagStdDevs Standard deviations [x, y, theta] when only one AprilTag is visible
         * @param multiTagStdDevs  Standard deviations [x, y, theta] when multiple AprilTags are visible
         */
        private CameraProperties(String name,
                Transform3d transform,
                Matrix<N3, N1> singleTagStdDevs,
                Matrix<N3, N1> multiTagStdDevs) {
            this.name = name;
            this.transform = transform;
            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevs;
        }

        /**
         * Returns the PhotonVision camera name for this camera.
         *
         * @return Camera name string
         */
        public String getName() {
            return name;
        }

        /**
         * Returns the 3D transform from the robot center to this camera.
         *
         * @return Robot-to-camera Transform3d
         */
        public Transform3d getTransform() {
            return transform;
        }

        /**
         * Returns the measurement standard deviations used when a single AprilTag is detected.
         *
         * @return Standard deviation matrix [x, y, theta] for single-tag estimation
         */
        public Matrix<N3, N1> getSingleTagStdDevs() {
            return singleTagStdDevs;
        }

        /**
         * Returns the measurement standard deviations used when multiple AprilTags are detected.
         *
         * @return Standard deviation matrix [x, y, theta] for multi-tag estimation
         */
        public Matrix<N3, N1> getMultiTagStdDevs() {
            return multiTagStdDevs;
        }
    }
}
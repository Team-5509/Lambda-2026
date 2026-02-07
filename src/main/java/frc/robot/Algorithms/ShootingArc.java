package frc.robot.Algorithms;

import edu.wpi.first.math.geometry.Pose2d;

public class ShootingArc {
    public class Constants {
        public static final double GoalConeX = 158.32;
        public static final double GoalConeY = 181.56;
        
    
    }
    public class TestShooting1 {
        public TestShooting1() {

        }
        public static double getAngle(Pose2d robotPose) {
            double dx= Constants.GoalConeX-robotPose.getX();
            double dy= Constants.GoalConeY-robotPose.getY();
            return Math.atan2(dy, dx);
        }
        
    }
}

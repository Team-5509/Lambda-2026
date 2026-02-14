package frc.robot.Algorithms;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootingArc {
    public class Constants {
        public static final double GoalConeX = 158.32 *0.0254;
        public static final double GoalConeY = 181.56 * 0.0254;
        
    
    }

    public class TestShooting1 {
        public TestShooting1() {

        }
        public static double getDistance(Pose2d robotPose) {
            double dx= Constants.GoalConeX-robotPose.getX();
            double dy= Constants.GoalConeY-robotPose.getY();
             double distance = Math.sqrt(dx*dx + dy*dy);
             SmartDashboard.putNumber("ShootingArc/DistanceToGoal", distance);
             return distance;
        }
        public static double getAngle(Pose2d robotPose) {
            double dx= Constants.GoalConeX-robotPose.getX();
            double dy= Constants.GoalConeY-robotPose.getY();
            double angle = Math.atan2(dy, dx);
            SmartDashboard.putNumber("ShootingArc", angle);
            return angle;
            
        }
        
    }
}

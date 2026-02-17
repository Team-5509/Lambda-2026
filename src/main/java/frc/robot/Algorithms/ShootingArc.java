package frc.robot.Algorithms;

import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootingArc {

    public class Constants {
        public static final double BlueHubY = 158.32 *0.0254;
        public static final double BlueHubX  = 181.56 *0.0254;
        public static final double RedHubY = 158.32 * 0.0254;
        public static final double RedHubX = 468.56 * 0.0254;
    }
    
        private double HubX;
        private double HubY;
        // TODO: verify the robot height of the shooter from the ground 
        private double dZ = 72 - 21.5; //different of height between hub and the shooter
        private final double gravity_in_ss = 386.088;
        public ShootingArc() {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
                HubX = Constants.RedHubX;
                HubY = Constants.RedHubY;
            }
            else{
                HubX = Constants.BlueHubX;
                HubY = Constants.BlueHubY;
            }
        }
        public double getDistanceToHub(Pose2d robotPose) {
            double dx= HubX-robotPose.getX();
            double dy= HubY-robotPose.getY();
             double distance = Math.sqrt(dx*dx + dy*dy);
             SmartDashboard.putNumber("ShootingArc/DistanceToGoal", distance);
             return distance;
        }
        public double getYawToHub(Pose2d robotPose) {
            double dx= HubX-robotPose.getX();
            double dy= HubY-robotPose.getY();
            double angle = Math.atan2(dy, dx);
            SmartDashboard.putNumber("ShootingArc", Math.toDegrees(angle));
            return angle;
            
        }

        /**
         * computes the hood angle which is the angle of the shooter in inches
         * @param shooterWheelSpeed get that in feet per second 
         * @param robotPose
         * @return angle in radians
         */
        public double getHoodAngleCalc(double shooterWheelSpeed, Pose2d robotPose) {
            double v0 = shooterWheelSpeed * 12;
            double yaw = getYawToHub(robotPose);
            // double elev
            double R = getDistanceToHub(robotPose);

            double v2 = v0 * v0;
            double disc = v2*v2 - gravity_in_ss * (gravity_in_ss * R * R + 2 * dZ * v2);
            double sqrt_disc = Math.sqrt(disc);
            double elev_low = Math.atan(v2 - sqrt_disc);
            double elev_high = Math.atan((v2 + sqrt_disc)/(gravity_in_ss * R));

            // TODO clamp elevation value to possible values for shooter 

            
            return elev_low;
        }


        public double solveShotWithDrag(Pose2d robotPose, double k){
            return 0;
        }
    }


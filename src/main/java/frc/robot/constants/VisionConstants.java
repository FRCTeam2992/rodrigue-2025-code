package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConstants {
    public static double goalX = 8.23;          // Meters
    public static double goalY = goalX / 2.0;   // Meters
    public static double goalRadius = .6096;     // Meters
    public static double limeLightOffset = 0.295;  // 11.593 inches = .295 meters
    public static double turretOffset = 0.0699;   // 2.75 inches = 0.0699 meters
    public static Pose2d goalPose = new Pose2d(goalX, goalY, new Rotation2d(0.0));
}

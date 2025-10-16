package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;

public class DriveConstants {

    // Drive Joystick Variables
    public static final boolean isFieldCentric = true;
    public static final boolean isVelocityControlled = true;
    public static final boolean isGyroCorrected = true;
    public static final double joystickDeadband = 0.1;
    public static double joystickXYSmoothFactor = 0.5;
    public static double joystickRotationSmoothFactor = 0.5;
    public static double joystickRotationInverseDeadband = 0.14;

    // Swerve Gyro Correction
    public static final double driveGyroP = 0.005;

    public static class DriveMotors {
        // Motor Base Config
        public static final boolean inverted = true;
        public static final int currenLimit = 30;
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final double closedLoopRampRate = 0.0;
        public static final double openLoopRampRate = 0.0;

        // Motor PID
        public static final double driveP = 0.0003;
        public static final double driveI = 0.0;
        public static final double driveD = 0.0;
        public static final double driveF = 0.0002;

    }

    public static class TurnMotors {
        public static final boolean inverted = true;
        public static final int currenLimit = 30;
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final double closedLoopRampRate = 0.0;
        public static final double openLoopRampRate = 0.0;

        // Motor PID
        public static final double turnP = 0.008;
        public static final double turnI = 0.0;
        public static final double turnD = 0.0;
        public static final double turnF = 0.00005;
    }

    // Swerve Wheels and Gear Ratio
    public static final double driveGearRatio = (40.0 / 16.0) * (18.0 / 26.0) * (60.0 / 15.0);
    public static final double driveWheelDiameter = 0.1016;

    // Analog Encoder Offsets (Degrees) - Opposite of Raw Reading - Bevel Gear to
    // Right relative to the intake
    public static final double frontLeftOffset = 0.0;
    public static final double frontRightOffset = -0.0;
    public static final double rearLeftOffset = 0.0;
    public static final double rearRightOffset = 0.0;

    // Max Swerve Speed (Velocity Control)
    public static final double swerveMaxSpeed = 2; // (Meters per Second)(4.5-normal, 2.0-slow)

    // Swerve Module Translations
    public static final Translation2d frontLeftLocation = new Translation2d(0.31115, 0.2794);
    public static final Translation2d frontRightLocation = new Translation2d(0.31115, -0.2794);
    public static final Translation2d rearLeftLocation = new Translation2d(-0.31115, 0.2794);
    public static final Translation2d rearRightLocation = new Translation2d(-0.31115, -0.2794);

    // Length and Width of the Robot in Meters (Inches: 22.0 x 24.5)
    public static final double swerveWidth = 0.5588;
    public static final double swerveLength = 0.6223;

}

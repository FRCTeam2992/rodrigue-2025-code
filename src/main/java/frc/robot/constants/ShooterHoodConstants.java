package frc.robot.constants;

public class ShooterHoodConstants {
    // the tooth to tooth of the hood 
    public static final double hoodAngleRatio = 38.000 / 520.000;
    public static final double hoodEncoderAngleRatio = 520.000 / 38.000;
    // the max and min of the encoder values not the hood angles
    public static final double minHoodPosition = -153.0;
    public static final double minHoodPZone = -100.0;

    public static final double maxHoodPosition = 153.0;
    public static final double maxHoodPZone = 100.0;

    public static final double hoodPValueBottom = 0.0045;
    public static final double hoodPValueTop = 0.0072;
    public static final double hoodEncoderOffset = -28.9 + 180;

    public static final double cameraHeight = 41.25;
    public static final double goalHeight = 102;
    public static final double distanceTest = 120;
    public static final double cameraAngle = 35;

    public static final double hoodP = .007;
    public static final double hoodI = 0.02; 
    public static final double hoodD = 0.0001;
    public static final double hoodF = 0.02; 

    public static final double hoodTolerance = 10;

    public static final double defaultHoodPosition = 0;
}

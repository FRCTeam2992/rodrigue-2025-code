package frc.robot.constants;

public class TurretConstants {
    public static final double turretP = 0.06;
    public static final double turretI = 0.012;
    public static final double turretD = 1.5;
    public static final double turretF = -0.02; // Need to push against CF spring in neg dir
    
    public static final double turretTolerance = 1.0;

    public static final int turretEncoderOffset = -12;
    public static final int turretRobotOffset = 70;

    public static final int turretMinEnd = 35; // 45
    public static final int turretMaxEnd = 320; // 298
    public static final int turretMinRumble = 50; // 60
    public static final int turretMaxRumble = 305; // 283
    public static final int turretMaxSlowZone = 245; // 223
    public static final int turretMinSlowZone = 110; // 100
    public static final double turretJoystickDeadband = .75;
    public static final double turretDefaultAngle = 180;
    public static final double turretGearRatio = (240.0 / 18.0) * (24.0 / 18.0);
    // public static final double turretGearRatio = 40.0 / 2.3;  // Faked to make angles work!
}

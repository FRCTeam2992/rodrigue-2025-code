package frc.robot.constants;

public class ShooterConstants {
    public static final double defaultMainShooterSpeed = 1000; // 2700;
    public static final double defaultSecondaryShooterSpeed = 1000; // 3300;
    public static final double shooterMainRPMTolerance = 200.0;
    public static final double shooterSecondaryRPMTolerance = 150.0;
    public static final double shooterSpeedTolerance = 100;
    public static final double minRPMThreshold = 250;

    public static final double mainShooterGearRatio = 0.75;
    public static final double secondaryShooterGearRatio = 1.0;
    public static final double minutesToSeconds = 60.0;
    
    public static class PIDMain {
        public static final double kP = 0.04;
        public static final double kI = 0.0;
        public static final double kD = 0.00025;
        public static final double kV = 0.01;
    }

    public static class PIDSecondary {
        public static final double kP = 0.04;
        public static final double kI = 0.0;
        public static final double kD = 0.00015;
        public static final double kV = 0.01;
    }
}

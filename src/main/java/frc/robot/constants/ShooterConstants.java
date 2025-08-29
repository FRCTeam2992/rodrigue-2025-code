package frc.robot.constants;

public class ShooterConstants {
    public static final double defaultMainShooterSpeed = 2700;
    public static final double defaultSecondaryShooterSpeed = 3300;
    public static final double shooterMainRPMTolerance = 200.0;
    public static final double shooterSecondaryRPMTolerance = 150.0;
    public static final double shooterSpeedTolerance = 100;
    
    public static class PIDMain {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 1.5;
        public static final double kV = 0.055;
    }

    public static class PIDSecondary {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 1.5;
        public static final double kV = 0.052;
    }
}

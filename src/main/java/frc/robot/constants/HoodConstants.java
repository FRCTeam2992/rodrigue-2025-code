package frc.robot.constants;

public class HoodConstants {
    public static final double hoodEncoderOffset = 0.0;
    public static final double hoodGearRatio = 1.0;
    public static final double hoodCANcoderMagnetOffset = -0.05;

    public static class Limits {
        public static final double hardStopMin = 12.0;
        public static final double softStopMin = 60.0;
        public static final double softStopMax = 300.0;
        public static final double hardStopMax = 340.0;
    }

    public static class PID {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        public static final double tolerance = 1.0;
        public static final double integratorMin = 0.0;
        public static final double integratorMax = 0.0;
    }
}

package frc.robot.constants;

public class TurretConstants {
    public static final double turretEncoderOffset = 0.0;
    public static final double turretDefaultAngle = 0.0;
    public static final double turretGearRatio = (240.0 / 18.0) * (24.0 / 18.0);
    public static final double turretRobotOffset = 68.5;
    public static final double turretCANcoderMagnetOffset = -0.038;

    public static class Limits {
        public static final double hardStopMin = 55.0;
        public static final double softStopMin = 85.0;
        public static final double softStopMax = 280.0;
        public static final double hardStopMax = 310.0;
        public static final double maxSlowModePower = 0.03;
        public static final double minSlowModePower = -0.07;
        public static final double maxPower = 0.08;
        public static final double minPower = -0.11;
    }

    public static class PID {
        public static final double kP = 0.06;
        public static final double kI = 0.012;
        public static final double kD = 1.5;
        public static final double kF = -0.02; // Need to push against CF spring in neg dir
        public static final double tolerance = 1.0;
        public static final double integratorMin = -0.18;
        public static final double integratorMax = 0.14;
    }
}

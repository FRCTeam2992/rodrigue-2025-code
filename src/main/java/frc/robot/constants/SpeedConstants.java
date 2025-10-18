package frc.robot.constants;

public class SpeedConstants {
    public static class ManualIntake {
        public static final double intakePower = 0.2;
        public static final double feederPower = 0.6;
        public static final double funnelPower = 0.4;
    }

    public static class AutoIntake {
        public static final double intakePower = 0.2;
        public static final double feederPower = 0.6;
        public static final double funnelPower = 0.4;
    }

    public static class DejamIntake {
        public static final double intakePower = -0.2;
        public static final double feederPower = -0.6;
        public static final double funnelPower = -0.6;
    }

    public static class Shooting {
        public static final double defaultMainShooterSpeed = 1000; // 2700;
        public static final double defaultSecondaryShooterSpeed = 1000; // 3300;
        public static final double shooterSpeedIncrementRPM = 500;

        public static final double feederPower = 0.6;
        public static final double funnelPower = 0.4;
    }

    public static class Turret {
        public static final double leftPower = -0.15;
        public static final double rightPower = 0.15;
    }

    public static class Hood {
        public static final double upPower = 0.25;
        public static final double downPower = -0.2;
        public static final double minPower = -0.2;
        public static final double maxPower = 0.25;
        public static final double minSlowModePower = -0.08;
        public static final double maxSlowModePower = 0.13;
    }
}

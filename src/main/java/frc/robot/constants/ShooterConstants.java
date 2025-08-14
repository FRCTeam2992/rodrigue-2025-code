package frc.robot.constants;

public class ShooterConstants {
    public static final double defaultMainShooterSpeed = 2700;
    public static final double defaultSecondaryShooterSpeed = 3300;
    public static final int shooterEncoderPulses = 2048;
    public static final double shooterSpeedTolerance = 100;
    
    public static final double shooterPIDMainP = 0.1;
    public static final double shooterPIDMainI = 0.0;
    public static final double shooterPIDMainD = 1.5;
    public static final double shooterPIDMainF = 0.055;

    public static final double shooterPIDSecondaryP = 0.1;
    public static final double shooterPIDSecondaryI = 0.0;
    public static final double shooterPIDSecondaryD = 1.5;
    public static final double shooterPIDSecondaryF = 0.052;
}

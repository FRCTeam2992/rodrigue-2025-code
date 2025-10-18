// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DebugConstants;
import frc.robot.constants.Devices;
import frc.robot.constants.HoodConstants;

public class Hood extends SubsystemBase {
    public enum HoodMode {
        ManualMove("Manual Move"),
        MoveToPosition("Move to Position"),
        Stopped("Stopped");

        public String displayName;

        private HoodMode(String name) {
            this.displayName = name;
        }
    }

    private HoodMode mode = HoodMode.Stopped;

    // Hardware
    private final SparkMax hoodMotor;
    private final CANcoder hoodAbsEncoder;

    // Hardware Configuration
    private final SparkMaxConfig hoodMotorConfiguration;
    private final CANcoderConfiguration hoodAbsEncoderConfiguration;

    // Signals
    private final StatusSignal<Angle> hoodAbsEncoderPosition;

    // Hood PID Controller
    public PIDController hoodRotate;
    double pidPower = 0.0;              // Last commanded PID power

    // Angle the hood was last targeted to turn to
    public double hoodTargetAngle = HoodConstants.Limits.softStopMin;

    private int dashboardCounter = 0;
    public double hoodTarget = 180.0;
    private double currentOutputPower = 0.0;

    public Hood() {
        // Hood Motors
        hoodMotorConfiguration = setupMotorConfiguration();
        hoodMotor = new SparkMax(Devices.CANDeviceAddress.ShooterHoodMotor.id, MotorType.kBrushed);
        applyMotorConfiguration();

        hoodAbsEncoderConfiguration = setupAbsEncoderConfiguration();
        hoodAbsEncoder = new CANcoder(Devices.CANDeviceAddress.ShooterHoodCANCoder.id);
        applyAbsEncoderConfiguration();
        addChild("Hood Abs Enc", hoodAbsEncoder);

        SmartDashboard.putString("Hood State", "stopped");

        hoodAbsEncoderPosition = hoodAbsEncoder.getPosition();

        // Hood PID Controller
        hoodRotate = new PIDController(
            HoodConstants.PID.kP,
            HoodConstants.PID.kI,
            HoodConstants.PID.kD);
        hoodRotate.setTolerance(HoodConstants.PID.tolerance);
        hoodRotate.disableContinuousInput();
        hoodRotate.setIntegratorRange(
            HoodConstants.PID.integratorMin,
            HoodConstants.PID.integratorMax);
    }

    @Override
    public void periodic() {
        hoodAbsEncoderPosition.refresh();

        switch (mode) {            
            case Stopped:
                hoodMotor.set(0.0);
                break;
            case ManualMove:
                hoodMotor.set(clampManualSpeed(currentOutputPower));
                break;
            case MoveToPosition:
                hoodMotor.set(0.0);
                break;
        }

        SmartDashboard.putNumber("Hood CanCoder Real Deg", getCanCoderRealAngle().in(Degrees));
        if (DebugConstants.Logging.enableHood && ++dashboardCounter >= 5) {
            // Update Dashboard
            SmartDashboard.putNumber("Hood Raw CanCoder Deg", hoodAbsEncoderPosition.getValue().in(Degrees));
            SmartDashboard.putNumber("Hood Target", hoodTargetAngle);
            SmartDashboard.putString("Hood Mode", mode.displayName);

            dashboardCounter = 0;
        }
    }

    public void stopHood() {
        SmartDashboard.putString("Hood State", "stopped");
        this.currentOutputPower = 0.0;
        this.mode = HoodMode.Stopped;
    }

    public void setHoodSpeed(double speed) {
        this.currentOutputPower = speed;
        this.mode = HoodMode.ManualMove;
    }

    private double clampManualSpeed(double speed) {
        double setSpeed = speed;
        double encoderDegrees = unwrapAngle(getCanCoderRealAngle().in(Degrees));

        String state = "free movement";

        if (setSpeed > 0.0 && encoderDegrees >= HoodConstants.Limits.softStopMax) {
            state = "max-side slow";
            setSpeed = Math.min(frc.robot.constants.SpeedConstants.Hood.maxSlowModePower, setSpeed);
        }

        if (setSpeed < 0.0 && encoderDegrees <= HoodConstants.Limits.softStopMin) {
            state = "min-side slow";
            setSpeed = Math.max(frc.robot.constants.SpeedConstants.Hood.minSlowModePower, setSpeed);
        }

        if ((setSpeed > 0.0 && encoderDegrees >= HoodConstants.Limits.hardStopMax)
                || (setSpeed < 0.0 && encoderDegrees < HoodConstants.Limits.hardStopMin)) {
            state = "hard stop";
            setSpeed = 0.0;
        }
        SmartDashboard.putString("Hood State", state);
        setSpeed = MathUtil.clamp(setSpeed, frc.robot.constants.SpeedConstants.Hood.minPower, frc.robot.constants.SpeedConstants.Hood.maxPower);
        return setSpeed;
    }

    // public void goToAngle(double angle) {
    //     hoodTargetAngle = angle;          // Save the angle that was last targeted

    //     // SmartDashboard.putNumber("HoodToAngle Angle", angle);
    //     angle = angleOverlap(angle + Constants.hoodRobotOffset);
    //     angle = Math.min(angle, Constants.hoodMaxEnd);
    //     angle = Math.max(angle, Constants.hoodMinEnd);
        
        
    //     // if (Math.abs(angle - getHoodAngleRaw()) > Constants.hoodTolerance) {
    //     //     hoodRotate.setSetpoint(angle);
    //     // }
    //     // if (Math.abs(angle - getHoodAngleRaw()) > 20.0) {
    //     //     hoodRotate.reset();
    //     // }
        
    //     // power = 0.0;
    //     // pidPower = hoodRotate.calculate(getHoodAngleRaw());
    //     // pidPower += Constants.hoodF;
    
    //     // pidPower = MathUtil.clamp(pidPower, -.50, 0.46);
        
    //     // SmartDashboard.putNumber("HoodToAngle Speed", pidPower);
        
    //     // Convert angle to Falcon encoder clicks
    //     // angle -= Constants.hoodRobotOffset;
    //     double motorTarget = angle * 2048.0 * Constants.hoodGearRatio / 360.0;

    //     // SmartDashboard.putNumber("Hood target ticks", motorTarget);

    //     //setHoodSpeed(pidPower);
    //     hoodMotor.set(ControlMode.MotionMagic, motorTarget, DemandType.ArbitraryFeedForward, Constants.hoodF);
    // }

    public Angle getCanCoderRealAngle() {
        // FIXME: figure out what this needs to be
        double position = hoodAbsEncoderPosition.getValue().in(Degrees) + HoodConstants.hoodEncoderOffset;
        return Degrees.of(unwrapAngle(position));
    }

    // public static double getHoodAngleRaw() {
    //     return unwrapAngle(getCanCoderRealDegrees() * 40.0 /  Constants.hoodGearRatio);       // Adjust for gear ratio of abs encoder
    // }
    
    public static double unwrapAngle(double tempAngle) {
        while (tempAngle > 360) {
            tempAngle -= 360;
        } 
        while (tempAngle < 0) {
            tempAngle += 360;
        }
        return tempAngle;
    }

    private void applyMotorConfiguration() {
        this.hoodMotor.configure(hoodMotorConfiguration, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    private void applyAbsEncoderConfiguration() {
        hoodAbsEncoder.getConfigurator().apply(hoodAbsEncoderConfiguration);
    }

    private SparkMaxConfig setupMotorConfiguration() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.smartCurrentLimit(15);
        return config;
    }

    private CANcoderConfiguration setupAbsEncoderConfiguration() {
        return new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(HoodConstants.hoodCANcoderMagnetOffset)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DebugConstants;
import frc.robot.constants.Devices;
import frc.robot.constants.TurretConstants;

public class Turret extends SubsystemBase {
    public enum TurretMode {
        ManualMove,
        MoveToPosition,
        Stopped;
    }

    private TurretMode mode = TurretMode.Stopped;

    // Hardware
    private final TalonFX turretMotor;
    private final CANcoder turretAbsEncoder;

    // Hardware Configuration
    private final TalonFXConfiguration turretMotorConfiguration;
    private final CANcoderConfiguration turretAbsEncoderConfiguration;

    // Signals
    private final StatusSignal<Angle> turretMotorEncoderPosition;
    private final StatusSignal<Angle> turretMotorRotorPosition;
    private final StatusSignal<Double> turretMotorClosedLoopError;
    private final StatusSignal<Angle> turretAbsEncoderPosition;

    // Control Request
    private final DutyCycleOut manualControlRequest = new DutyCycleOut(0.0);
    private ControlRequest currentControlRequest;

    // Turret PID Controller
    public PIDController turretRotate;
    double pidPower = 0.0;              // Last commanded PID power

    public double turretTargetAngle = TurretConstants.turretDefaultAngle;              // Angle the turret was last targeted to turn to

    private int dashboardCounter = 0;
    public double turretTarget = 180.0;
    private double currentOutputPower = 0.0;

    public Turret() {
        // Turret Motors
        turretMotorConfiguration = setupMotorConfiguration();
        turretMotor = new TalonFX(Devices.CANDeviceAddress.TurretMotor.id);
        applyMotorConfiguration();
        addChild("Turret Motor", turretMotor);

        turretMotorEncoderPosition = turretMotor.getPosition();
        turretMotorRotorPosition = turretMotor.getRotorPosition();
        turretMotorClosedLoopError = turretMotor.getClosedLoopError();

        turretAbsEncoderConfiguration = setupAbsEncoderConfiguration();
        turretAbsEncoder = new CANcoder(Devices.CANDeviceAddress.TurretCANCoder.id);
        applyAbsEncoderConfiguration();
        addChild("Turret Abs Enc", turretAbsEncoder);

        turretAbsEncoderPosition = turretAbsEncoder.getPosition();

        // Turret PID Controller
        turretRotate = new PIDController(
            TurretConstants.PID.kP,
            TurretConstants.PID.kI,
            TurretConstants.PID.kD);
        turretRotate.setTolerance(TurretConstants.PID.tolerance);
        turretRotate.disableContinuousInput();
        turretRotate.setIntegratorRange(
            TurretConstants.PID.integratorMin,
            TurretConstants.PID.integratorMax);

        setTurretFalconEncoder();
    }

    @Override
    public void periodic() {
        if (hasTurretFalconReset()) {
            setTurretFalconEncoder();
        }

        turretMotorEncoderPosition.refresh();
        turretMotorClosedLoopError.refresh();
        turretAbsEncoderPosition.refresh();

        currentControlRequest = manualControlRequest.withOutput(0.0);
        switch (mode) {            
            case Stopped:
                currentControlRequest = manualControlRequest.withOutput(0.0);
                break;
            case ManualMove:
                currentControlRequest = manualControlRequest.withOutput(clampManualSpeed(currentOutputPower));
                break;
            case MoveToPosition:
                currentControlRequest = manualControlRequest.withOutput(0.0);
                break;
        }
        turretMotor.setControl(currentControlRequest);

        if (DebugConstants.Logging.enableTurret && ++dashboardCounter >= 5) {
            // Update Dashboard
            SmartDashboard.putNumber("Turret CanCoder Real", getCanCoderRealDegrees().in(Degrees));
            SmartDashboard.putNumber("Turret RobotCentric Angle", unwrapAngle(getRobotCentricTurretAngleDegrees()));
            SmartDashboard.putNumber("Turret Falcon Real", unwrapAngle(getFalconRealDegrees().in(Degrees)));

            SmartDashboard.putNumber("Turret Falcon Rotor Pos", turretMotorRotorPosition.getValue().in(Degrees));
            SmartDashboard.putNumber("Turret Target", turretTargetAngle);
            // SmartDashboard.putNumber("Turret Raw", getTurretAngleRaw());

            dashboardCounter = 0;
        }
    }

    public void stopTurret() {
        SmartDashboard.putString("Turret State", "stopped");
        this.currentOutputPower = 0.0;
        this.mode = TurretMode.Stopped;
    }

    public void setTurretSpeed(double speed) {
        this.currentOutputPower = speed;
        this.mode = TurretMode.ManualMove;
    }

    private double clampManualSpeed(double speed) {
        double setSpeed = speed;
        double falconRealDegrees = unwrapAngle(getFalconRealDegrees().in(Degrees)); 

        String state = "free movement";

        if (setSpeed > 0.0 && falconRealDegrees >= TurretConstants.Limits.softStopMax) {
            state = "max-side slow";
            setSpeed = Math.min(TurretConstants.Limits.maxSlowModePower, setSpeed);
        }

        if (setSpeed < 0.0 && falconRealDegrees <= TurretConstants.Limits.softStopMin) {
            state = "min-side slow";
            setSpeed = Math.max(TurretConstants.Limits.minSlowModePower, setSpeed);
        }

        if ((setSpeed > 0.0 && falconRealDegrees >= TurretConstants.Limits.hardStopMax)
                || (setSpeed < 0.0 && falconRealDegrees < TurretConstants.Limits.hardStopMin)) {
            state = "hard stop";
            setSpeed = 0.0;
        }
        SmartDashboard.putString("Turret State", state);
        setSpeed = MathUtil.clamp(setSpeed, TurretConstants.Limits.minPower, TurretConstants.Limits.maxPower);
        return setSpeed;
    }

    // public void goToAngle(double angle) {
    //     turretTargetAngle = angle;          // Save the angle that was last targeted

    //     // SmartDashboard.putNumber("TurretToAngle Angle", angle);
    //     angle = angleOverlap(angle + Constants.turretRobotOffset);
    //     angle = Math.min(angle, Constants.turretMaxEnd);
    //     angle = Math.max(angle, Constants.turretMinEnd);
        
        
    //     // if (Math.abs(angle - getTurretAngleRaw()) > Constants.turretTolerance) {
    //     //     turretRotate.setSetpoint(angle);
    //     // }
    //     // if (Math.abs(angle - getTurretAngleRaw()) > 20.0) {
    //     //     turretRotate.reset();
    //     // }
        
    //     // power = 0.0;
    //     // pidPower = turretRotate.calculate(getTurretAngleRaw());
    //     // pidPower += Constants.turretF;
    
    //     // pidPower = MathUtil.clamp(pidPower, -.50, 0.46);
        
    //     // SmartDashboard.putNumber("TurretToAngle Speed", pidPower);
        
    //     // Convert angle to Falcon encoder clicks
    //     // angle -= Constants.turretRobotOffset;
    //     double motorTarget = angle * 2048.0 * Constants.turretGearRatio / 360.0;

    //     // SmartDashboard.putNumber("Turret target ticks", motorTarget);

    //     //setTurretSpeed(pidPower);
    //     turretMotor.set(ControlMode.MotionMagic, motorTarget, DemandType.ArbitraryFeedForward, Constants.turretF);
    // }

    public void setTurretFalconEncoder() {
        turretMotor.setPosition(getCanCoderRealDegrees());
    }

    public Angle getFalconRealDegrees() {
        return turretMotorEncoderPosition.getValue();
    }

    public boolean hasTurretFalconReset() {
        return turretMotor.hasResetOccurred();
    }

    public Angle getCanCoderRealDegrees() {
        double position = turretAbsEncoderPosition.getValue().in(Degrees) + TurretConstants.turretEncoderOffset;
        position *= 40.0 / TurretConstants.turretGearRatio;


        return Degrees.of(unwrapAngle(position));
    }

    // public static double getTurretAngleRaw() {
    //     return unwrapAngle(getCanCoderRealDegrees() * 40.0 /  Constants.turretGearRatio);       // Adjust for gear ratio of abs encoder
    // }
    
    public double getRobotCentricTurretAngleDegrees() {
        return unwrapAngle(getFalconRealDegrees().in(Degrees) - TurretConstants.turretRobotOffset);
    }

    public static double unwrapAngle(double tempAngle) {
        while (tempAngle > 360) {
            tempAngle -= 360;
        } 
        while (tempAngle < 0) {
            tempAngle += 360;
        }
        return tempAngle;
    }

    public double getClosedLoopError() {
        return turretMotorClosedLoopError.getValueAsDouble();
    }

    private void applyMotorConfiguration() {
        turretMotor.getConfigurator().apply(turretMotorConfiguration);
    }

    private void applyAbsEncoderConfiguration() {
        turretAbsEncoder.getConfigurator().apply(turretAbsEncoderConfiguration);
    }

    private TalonFXConfiguration setupMotorConfiguration() {
        return new TalonFXConfiguration()
            // TODO: Add current limits?
            // .withCurrentLimits(new CurrentLimitsConfigs()
            //     .withSupplyCurrentLimit(null)
            //     .withSupplyCurrentLimitEnable(false)
            //     .withStatorCurrentLimit(null)
            //     .withStatorCurrentLimitEnable(false))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(TurretConstants.turretGearRatio));
        
        // If needed, we can update these. Config change is documented in migration guide:
        // https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/status-signals-guide.html#changing-update-frequency-status-frame-period
        // turretFalcon.setStatusFramePeriod(3, 255);
        // turretFalcon.setStatusFramePeriod(4, 254);
        // turretFalcon.setStatusFramePeriod(8, 253);
        // turretFalcon.setStatusFramePeriod(10, 252);
        // turretFalcon.setStatusFramePeriod(12, 251);
        // turretFalcon.setStatusFramePeriod(13, 250);
        // turretFalcon.setStatusFramePeriod(14, 249);
    }

    private CANcoderConfiguration setupAbsEncoderConfiguration() {
        // This is no longer needed: https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/feature-replacements-guide.html#sensor-initialization-strategy
        // turretAbsEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        return new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(TurretConstants.turretCANcoderMagnetOffset));
    }
}
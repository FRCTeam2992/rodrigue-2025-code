package frc.lib.drive.swerve;



import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;

public class SwerveModuleNeoNeo {

    // Saved Variables
    private SparkMax driveMotor;
    private SparkMax turnMotor;
    private CANcoder encoderInput;
    private StatusSignal<Angle> encoderAbsPosition;
    private double encoderOffset;
    private PIDController turnPID;
    private double wheelDiameter;
    private double wheelGearRatio;
    private double maxDriveSpeed;

    public SwerveModuleNeoNeo(SparkMax driveMotor, SparkMax turnMotor, CANcoder encoderInput, double encoderOffset,
            PIDController turnPID, double wheelDiameter, double wheelGearRatio, double maxDriveSpeed) {
        // Saved Variables
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.encoderInput = encoderInput;
        this.encoderAbsPosition = this.encoderInput.getAbsolutePosition();
        this.encoderOffset = encoderOffset;
        this.turnPID = turnPID;
        this.wheelDiameter = wheelDiameter;
        this.wheelGearRatio = wheelGearRatio;
        this.maxDriveSpeed = maxDriveSpeed;
    }

    public void setDriveSpeed(double speed) {
        driveMotor.set(speed);
    }

    public void setTurnSpeed(double speed) {
        turnMotor.set(speed);
    }

    public void stop() {
        setDriveSpeed(0.0);
        setTurnSpeed(0.0);
    }

    public void setTurnAngle(double degrees) {
        degrees = Math.min(Math.max(degrees, -180.0), 180.0);
        setTurnSpeed(turnPID.calculate(getEncoderAngle(), degrees));
    }

    public void setVelocityMeters(double speed) {
        double rpm = (speed * wheelGearRatio * 60) / (wheelDiameter * Math.PI);
        driveMotor.getClosedLoopController().setReference(rpm, ControlType.kVelocity);
    }

    public void setDrive(double speed, double angle) {
        if (Math.abs(getEncoderAngle() - angle) > 90.0) {
            if (angle > 0) {
                angle -= 180.0;
            } else {
                angle += 180.0;
            }

            speed = -speed;
        }

        setDriveSpeed(speed);
        setTurnAngle(angle);
    }

    public void setDriveVelocity(double speedPercent, double angle) {
        double speed = speedPercent * maxDriveSpeed;

        if (Math.abs(getEncoderAngle() - angle) > 90.0) {
            if (angle > 0) {
                angle -= 180.0;
            } else {
                angle += 180.0;
            }

            speed = -speed;
        }

        setVelocityMeters(speed);
        setTurnAngle(angle);
    }

    public boolean atAngle() {
        return turnPID.atSetpoint();
    }

    public double getEncoderAngle() {
        double tempAngle = encoderAbsPosition.getValue().in(Degrees) - encoderOffset;

        tempAngle -= 180.0;

        if (tempAngle < -180.0) {
            tempAngle += 360.0;
        } else if (tempAngle > 180.0) {
            tempAngle -= 360.0;
        }

        return -tempAngle;
    }

    public void refreshEncoderPosition() {
        encoderAbsPosition.refresh();
    }

    public double getWheelSpeedMeters() {
        double RPM = driveMotor.getEncoder().getVelocity();

        double speed = (RPM * wheelDiameter * Math.PI) / (wheelGearRatio * 60);

        return speed;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelSpeedMeters(), Rotation2d.fromDegrees(getEncoderAngle()));
    }

    public void setState(SwerveModuleState state) {
        double angle = state.angle.getDegrees();
        double speed = state.speedMetersPerSecond;

        setVelocityMeters(speed);
        setTurnAngle(angle);
    }
}

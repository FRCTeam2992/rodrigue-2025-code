package frc.robot.subsystems;

import frc.robot.constants.Devices;
import frc.robot.constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 *
 */
public class Shooter extends SubsystemBase {
  private double mainShooterSetRPM = ShooterConstants.defaultMainShooterSpeed;
  private double secondaryShooterSetRPM = ShooterConstants.defaultSecondaryShooterSpeed;
  private boolean shooterCommanded = false;

  private int dashboardCounter = 0;

  private TalonFX mainShooterLeadMotor;
  private TalonFXConfiguration mainShooterLeadConfig;
  private TalonFX mainShooterFollowMotor;
  private TalonFXConfiguration mainShooterFollowConfig;
  private Follower followLeadMotor;
  private TalonFX secondaryShooterMotor;
  private TalonFXConfiguration secondaryShooterConfig;

  private DutyCycleOut manualControlRequest;
  private VelocityDutyCycle velocityControlRequest;

  /**
  *
  */
  public Shooter() {
    // Main shooter wheels
    mainShooterLeadMotor = new TalonFX(Devices.CANDeviceAddress.ShooterFlywheelLeader.id);
    mainShooterLeadConfig = setupMainShooterLeadMotorConfig();
    addChild("mainShooterLead", mainShooterLeadMotor);

    mainShooterFollowConfig = setupMainShooterFollowMotorConfig();
    mainShooterFollowMotor = new TalonFX(Devices.CANDeviceAddress.ShooterFlywheelFollower.id);
    addChild("mainShooterFollow", mainShooterFollowMotor);

    applyMainMotorConfigs();

    // Secondary (backspin) shooter wheels
    secondaryShooterConfig = setupSecondaryShooterMotorConfiguration();
    secondaryShooterMotor = new TalonFX(Devices.CANDeviceAddress.ShooterBackspin.id);
    addChild("secondaryShooterLead", secondaryShooterMotor);

    applySecondaryMotorConfig();

    // Control requests
    followLeadMotor = new Follower(Devices.CANDeviceAddress.ShooterFlywheelLeader.id, true);
    manualControlRequest = new DutyCycleOut(0.0);
    velocityControlRequest = new VelocityDutyCycle(0.0);
  }

  @Override
  public void periodic() {
    if (mainShooterLeadMotor.hasResetOccurred() || mainShooterFollowMotor.hasResetOccurred()) {
      applyMainMotorConfigs();
    }
    if (secondaryShooterMotor.hasResetOccurred()) {
      applySecondaryMotorConfig();
    }

    mainShooterFollowMotor.setControl(followLeadMotor);

    if (++dashboardCounter >= 5) {
      publishTelemetry();
      dashboardCounter = 0;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  public void setMainShooterPower(double speed) {
    mainShooterLeadMotor.setControl(manualControlRequest.withOutput(speed));
  }

  public void setSecondaryShooterPower(double speed) {
    secondaryShooterMotor.setControl(manualControlRequest.withOutput(speed));
  }

  public void setMainShooterToTargetRPM() {
    // TODO: What are these numbers?
    double speed = (mainShooterSetRPM / 600.0) * 0.75;
    setMainShooterRawVelocity(speed);
  }

  private void setMainShooterRawVelocity(double velocity) {
    mainShooterLeadMotor.setControl(velocityControlRequest.withVelocity(velocity));
  }

  public void setSecondaryShooterToTargetRPM() {
    // TODO: What are these numbers?
    double speed = secondaryShooterSetRPM / 600.0;
    setSecondaryShooterRawVelocity(speed);
  }

  private void setSecondaryShooterRawVelocity(double velocity) {
    secondaryShooterMotor.setControl(velocityControlRequest.withVelocity(velocity));
  }

  public double getMainShooterRPM() {
    // TODO: What are these numbers?
    return (mainShooterLeadMotor.getVelocity().getValueAsDouble() * 600) / 0.75;
  }

  public double getSecondaryShooterRPM() {
    // TODO: What are these numbers?
    return (secondaryShooterMotor.getVelocity().getValueAsDouble() * 600);
  }

  public double getMainShooterTargetRPM() {
    return mainShooterSetRPM;
  }

  public void setMainShooterTargetRPM(double mainShooterSetSpeed) {
    this.mainShooterSetRPM = mainShooterSetSpeed;
  }

  public double getSecondaryShooterTargetRPM() {
    return secondaryShooterSetRPM;
  }

  public void setSecondaryShooterTargetRPM(double secondaryShooterSetSpeed) {
    this.secondaryShooterSetRPM = secondaryShooterSetSpeed;
  }

  public boolean atMainShooterRPM() {
    return (Math.abs(getMainShooterTargetRPM() - getMainShooterRPM()) < ShooterConstants.shooterMainRPMTolerance);
  }

  public boolean atSecondaryShooterRPM() {
    return (Math.abs(getSecondaryShooterTargetRPM() - getSecondaryShooterRPM()) < ShooterConstants.shooterSecondaryRPMTolerance);
  }

  public boolean atShooterRPM() {
    return (atMainShooterRPM() && atSecondaryShooterRPM());
  }

  public boolean isShooterCommanded() {
    return shooterCommanded;
  }

  public void setShooterCommanded(boolean shooterCommanded) {
    this.shooterCommanded = shooterCommanded;
  }

  public void reset() {
    setShooterCommanded(false);
  }

  private void publishTelemetry() {
    // Display the Shooter Set Speed and Current RPM
    SmartDashboard.putNumber("Main Shooter Set Speed", mainShooterSetRPM);
    SmartDashboard.putNumber("Main Shooter Current RPM", getMainShooterRPM());

    SmartDashboard.putNumber("Secondary Shooter Set Speed", secondaryShooterSetRPM);
    SmartDashboard.putNumber("Secondary Shooter Current RPM", getSecondaryShooterRPM());

    SmartDashboard.putBoolean("Main Shooter At Speed", atMainShooterRPM());
    SmartDashboard.putBoolean("Secondary Shooter At Speed", atSecondaryShooterRPM());
    SmartDashboard.putBoolean("Shooters at Speed", atShooterRPM());
  }

  private TalonFXConfiguration setupMainShooterLeadMotorConfig() {
    return new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40.0)
        .withSupplyCurrentLowerLimit(60.0)
        .withSupplyCurrentLowerTime(0.1)
        .withSupplyCurrentLimitEnable(true))
      .withSlot0(new Slot0Configs()
        .withKP(ShooterConstants.PIDMain.kP)
        .withKI(ShooterConstants.PIDMain.kI)
        .withKD(ShooterConstants.PIDMain.kD)
        .withKV(ShooterConstants.PIDMain.kV));
  }

  private TalonFXConfiguration setupMainShooterFollowMotorConfig() {
    return new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40.0)
        .withSupplyCurrentLowerLimit(60.0)
        .withSupplyCurrentLowerTime(0.1)
        .withSupplyCurrentLimitEnable(true))
      .withSlot0(new Slot0Configs()
        .withKP(ShooterConstants.PIDMain.kP)
        .withKI(ShooterConstants.PIDMain.kI)
        .withKD(ShooterConstants.PIDMain.kD)
        .withKV(ShooterConstants.PIDMain.kV));
  }

  private TalonFXConfiguration setupSecondaryShooterMotorConfiguration() {
    return new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40.0)
        .withSupplyCurrentLowerLimit(60.0)
        .withSupplyCurrentLowerTime(0.25)
        .withSupplyCurrentLimitEnable(true))
      .withSlot0(new Slot0Configs()
        .withKP(ShooterConstants.PIDSecondary.kP)
        .withKI(ShooterConstants.PIDSecondary.kI)
        .withKD(ShooterConstants.PIDSecondary.kD)
        .withKV(ShooterConstants.PIDSecondary.kV));
  }

  private void applyMainMotorConfigs() {
    mainShooterLeadMotor.getConfigurator().apply(mainShooterLeadConfig);
    mainShooterFollowMotor.getConfigurator().apply(mainShooterFollowConfig);
  }

  private void applySecondaryMotorConfig() {
    secondaryShooterMotor.getConfigurator().apply(secondaryShooterConfig);
  }  
}
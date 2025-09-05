package frc.robot.subsystems;

import frc.robot.constants.DebugConstants;
import frc.robot.constants.Devices;
import frc.robot.constants.ShooterConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
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
  public enum ShooterMode {
    Stopped("Stopped"),
    ManualSpin("Manual Spin"),
    Shooting("Shooting");

    public final String displayName;

    private ShooterMode(String displayName) {
      this.displayName = displayName;
    }
  }

  private ShooterMode mode = ShooterMode.Stopped;

  private double mainShooterSetRPM = ShooterConstants.defaultMainShooterSpeed;
  private double secondaryShooterSetRPM = ShooterConstants.defaultSecondaryShooterSpeed;

  private double mainShooterManualPower = 0.0;
  private double secondaryShooterManualPower = 0.0;

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
  private ControlRequest currentMainShooterRequest;
  private ControlRequest currentSecondaryShooterRequest;

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

    currentMainShooterRequest = manualControlRequest.withOutput(0.0);
    currentSecondaryShooterRequest = manualControlRequest.withOutput(0.0);

    switch (mode) {
      case Stopped:
        currentMainShooterRequest = manualControlRequest.withOutput(0.0);
        currentSecondaryShooterRequest = manualControlRequest.withOutput(0.0);
        break;
      case Shooting:
        double mainVelocity = calculateMotorVelocityRPSFromMechanismRPM(mainShooterSetRPM, ShooterConstants.mainShooterGearRatio);
        double secondaryVelocity = calculateMotorVelocityRPSFromMechanismRPM(secondaryShooterSetRPM, ShooterConstants.secondaryShooterGearRatio);
        currentMainShooterRequest = velocityControlRequest.withVelocity(mainVelocity);
        currentSecondaryShooterRequest = velocityControlRequest.withVelocity(secondaryVelocity);
        break;
      case ManualSpin:
        currentMainShooterRequest = manualControlRequest.withOutput(mainShooterManualPower);
        currentSecondaryShooterRequest = manualControlRequest.withOutput(secondaryShooterManualPower);
        break;        
    }

    mainShooterLeadMotor.setControl(currentMainShooterRequest);
    secondaryShooterMotor.setControl(currentSecondaryShooterRequest);
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

  public void setMode(ShooterMode newMode) {
    this.mode = newMode;
  }

  public void setMainShooterPower(double percentOutput) {
    this.mainShooterManualPower = MathUtil.clamp(percentOutput, -1.0, 1.0);
  }

  public void setSecondaryShooterPower(double percentOutput) {
    this.secondaryShooterManualPower = MathUtil.clamp(percentOutput, -1.0, 1.0);
  }

  public double getMainShooterRPM() {
    return calculateMechanismRPMFromMotorVelocityRPS(
      mainShooterLeadMotor.getVelocity().getValueAsDouble(),
      ShooterConstants.mainShooterGearRatio);
  }

  public double getSecondaryShooterRPM() {
    return calculateMechanismRPMFromMotorVelocityRPS(
      secondaryShooterMotor.getVelocity().getValueAsDouble(),
      ShooterConstants.secondaryShooterGearRatio);
  }

  public void setMainShooterTargetRPM(double mainShooterSetSpeed) {
    this.mainShooterSetRPM = mainShooterSetSpeed;
  }

  public void setSecondaryShooterTargetRPM(double secondaryShooterSetSpeed) {
    this.secondaryShooterSetRPM = secondaryShooterSetSpeed;
  }

  public void updateMainShooterTargetRPM(double velocityChangeRPM) {
    this.mainShooterSetRPM = mainShooterSetRPM + velocityChangeRPM;
  }

  public void updateSecondaryShooterTargetRPM(double velocityChangeRPM) {
    this.secondaryShooterSetRPM = secondaryShooterSetRPM + velocityChangeRPM;
  }

  public double getMainShooterTargetRPM() {
    return mainShooterSetRPM;
  }

  public double getSecondaryShooterTargetRPM() {
    return secondaryShooterSetRPM;
  }

  public boolean atShooterRPM() {
    return (atMainShooterRPM() && atSecondaryShooterRPM());
  }

  public void reset() {
    this.mode = ShooterMode.Stopped;
  }

  private double calculateMotorVelocityRPSFromMechanismRPM(double velocityRPM, double gearRatio) {
    // FIXME: What is the extra 10.0?
    return (velocityRPM / (ShooterConstants.minutesToSeconds * 10.0)) * gearRatio;
  }

  private double calculateMechanismRPMFromMotorVelocityRPS(double velocityRPS, double gearRatio) {
    // FIXME: What is the extra 10.0?
    return (velocityRPS * (ShooterConstants.minutesToSeconds * 10.0)) / gearRatio;
  }

  private boolean atMainShooterRPM() {
    return (Math.abs(getMainShooterTargetRPM() - getMainShooterRPM()) < ShooterConstants.shooterMainRPMTolerance);
  }

  private boolean atSecondaryShooterRPM() {
    return (Math.abs(getSecondaryShooterTargetRPM() - getSecondaryShooterRPM()) < ShooterConstants.shooterSecondaryRPMTolerance);
  }

  private void publishTelemetry() {
    // Display the Shooter Set Speed and Current RPM
    SmartDashboard.putString("Shooter: Mode", this.mode.displayName);
    SmartDashboard.putBoolean("Shooter: @Spd", atShooterRPM());

    SmartDashboard.putNumber("Shooter: Mn Tgt RPM", mainShooterSetRPM);
    SmartDashboard.putNumber("Shooter: Sc Tgt RPM", secondaryShooterSetRPM);

    if (DebugConstants.Logging.enableShooter) {
      SmartDashboard.putNumber("Shooter: Sc Curr RPM", getSecondaryShooterRPM());
      SmartDashboard.putNumber("Shooter: Mn Curr RPM", getMainShooterRPM());
  
      SmartDashboard.putBoolean("Shooter: Main @Spd", atMainShooterRPM());
      SmartDashboard.putBoolean("Shooter: Sec @Spd", atSecondaryShooterRPM());
  
      SmartDashboard.putNumber("Shooter: Mn Man Pwr", mainShooterManualPower);
      SmartDashboard.putNumber("Shooter: Sc Man Pwr", secondaryShooterManualPower);  
    }
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
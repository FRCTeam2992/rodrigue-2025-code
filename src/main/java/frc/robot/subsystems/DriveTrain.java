// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project....

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.swerve.SwerveController;
import frc.lib.drive.swerve.SwerveModuleNeoNeo;
import frc.robot.constants.DebugConstants;
import frc.robot.constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  // Drive Motors
  private final SparkMax frontLeftDrive;
  private final SparkMax frontLeftTurn;

  private final SparkMax frontRightDrive;
  private final SparkMax frontRightTurn;

  private final SparkMax rearLeftDrive;
  private final SparkMax rearLeftTurn;

  private final SparkMax rearRightDrive;
  private final SparkMax rearRightTurn;

  // Module Angle Encoders
  private final AnalogInput frontLeftEncoder;
  private final AnalogInput frontRightEncoder;
  private final AnalogInput rearLeftEncoder;
  private final AnalogInput rearRightEncoder;

  // Turn PID Controllers
  private final PIDController frontLeftController;
  private final PIDController frontRightController;
  private final PIDController rearLeftController;
  private final PIDController rearRightController;

  // Swerve Modules
  public final SwerveModuleNeoNeo frontLeftModule;
  public final SwerveModuleNeoNeo frontRightModule;
  public final SwerveModuleNeoNeo rearLeftModule;
  public final SwerveModuleNeoNeo rearRightModule;

  // Swerve Controller
  public final SwerveController swerveController;

  // Robot Gyro
  public AHRS navx;

  // Swerve Drive Kinematics
  public final SwerveDriveKinematics swerveDriveKinematics;

  // Swerve Pose
  public Pose2d latestSwervePose;

  // Motion Trajectories
  public Trajectory SlalomTrajectory;
  public Trajectory BarrelTrajectory;
  public Trajectory BounceTrajectory;
  public Trajectory GalacticSearchARedTrajectory;
  public Trajectory GalacticSearchABlueTrajectory;
  public Trajectory GalacticSearchBRedTrajectory;
  public Trajectory GalacticSearchBBlueTrajectory;
  public Trajectory PowerPortForward;
  public Trajectory PowerPortBackward;
  public Trajectory TeamNumberPath;
  public Trajectory CenterTrenchFiveTrajectory;
  public Trajectory RightTrenchFiveTrajectory;
  public Trajectory CenterTrenchThreeTrajectory;
  public Trajectory RightTrenchThreeTrajectory;
  public Trajectory CenterShieldGeneratorTrajectory;

  // DriveTrain Dashboard Update Counter
  private int dashboardCounter = 3; // first to relay to dashboard

  public DriveTrain() {
    // Drive Motors
    frontLeftDrive = new SparkMax(1, MotorType.kBrushless);
    frontLeftTurn = new SparkMax(2, MotorType.kBrushless);

    frontRightDrive = new SparkMax(3, MotorType.kBrushless);
    frontRightTurn = new SparkMax(4, MotorType.kBrushless);

    rearLeftDrive = new SparkMax(5, MotorType.kBrushless);
    rearLeftTurn = new SparkMax(6, MotorType.kBrushless);

    rearRightDrive = new SparkMax(7, MotorType.kBrushless);
    rearRightTurn = new SparkMax(8, MotorType.kBrushless);

    // Config the Drive Motors
    configDriveMotors();

    // Config the Turn Motors
    configTurnMotors();

    // Drive Encoders
    frontLeftEncoder = new AnalogInput(0);
    frontRightEncoder = new AnalogInput(1);
    rearLeftEncoder = new AnalogInput(2);
    rearRightEncoder = new AnalogInput(3);

    // Turn PID Controllers
    frontLeftController = new PIDController(DriveConstants.TurnMotors.turnP, DriveConstants.TurnMotors.turnI,
        DriveConstants.TurnMotors.turnD);
    frontLeftController.enableContinuousInput(-180.0, 180.0);

    frontRightController = new PIDController(DriveConstants.TurnMotors.turnP, DriveConstants.TurnMotors.turnI,
        DriveConstants.TurnMotors.turnD);
    frontRightController.enableContinuousInput(-180.0, 180.0);

    rearLeftController = new PIDController(DriveConstants.TurnMotors.turnP, DriveConstants.TurnMotors.turnI,
        DriveConstants.TurnMotors.turnD);
    rearLeftController.enableContinuousInput(-180.0, 180.0);

    rearRightController = new PIDController(DriveConstants.TurnMotors.turnP, DriveConstants.TurnMotors.turnI,
        DriveConstants.TurnMotors.turnD);
    rearRightController.enableContinuousInput(-180.0, 180.0);

    // Swerve Modules
    frontLeftModule = new SwerveModuleNeoNeo(frontLeftDrive, frontLeftTurn, frontLeftEncoder,
        DriveConstants.frontLeftOffset, frontLeftController, DriveConstants.driveWheelDiameter,
        DriveConstants.driveGearRatio,
        DriveConstants.swerveMaxSpeed);

    frontRightModule = new SwerveModuleNeoNeo(frontRightDrive, frontRightTurn, frontRightEncoder,
        DriveConstants.frontRightOffset, frontRightController, DriveConstants.driveWheelDiameter,
        DriveConstants.driveGearRatio,
        DriveConstants.swerveMaxSpeed);

    rearLeftModule = new SwerveModuleNeoNeo(rearLeftDrive, rearLeftTurn, rearLeftEncoder, DriveConstants.rearLeftOffset,
        rearLeftController, DriveConstants.driveWheelDiameter, DriveConstants.driveGearRatio,
        DriveConstants.swerveMaxSpeed);

    rearRightModule = new SwerveModuleNeoNeo(rearRightDrive, rearRightTurn, rearRightEncoder,
        DriveConstants.rearRightOffset, rearRightController, DriveConstants.driveWheelDiameter,
        DriveConstants.driveGearRatio,
        DriveConstants.swerveMaxSpeed);

    // Swerve Controller
    swerveController = new SwerveController(DriveConstants.swerveLength, DriveConstants.swerveWidth);

    // Robot Gyro
    navx = new AHRS(NavXComType.kMXP_SPI);

    // Swerve Drive Kinematics
    swerveDriveKinematics = new SwerveDriveKinematics(DriveConstants.frontLeftLocation,
        DriveConstants.frontRightLocation,
        DriveConstants.rearLeftLocation, DriveConstants.rearRightLocation);

  }

  @Override
  public void periodic() {

    if (DebugConstants.Logging.enableDrive && ++dashboardCounter >= 5) {
      // Display Module Angles
      SmartDashboard.putNumber("Drive: FL Angle", frontLeftModule.getEncoderAngle());
      SmartDashboard.putNumber("Drive: FR Angle", frontRightModule.getEncoderAngle());
      SmartDashboard.putNumber("Drive: RL Angle", rearLeftModule.getEncoderAngle());
      SmartDashboard.putNumber("Drive RR Angle", rearRightModule.getEncoderAngle());

      // Display Wheel Velocities
      SmartDashboard.putNumber("Drive: FL Velocity",
          frontLeftModule.getWheelSpeedMeters());
      SmartDashboard.putNumber("Drive: FR Velocity",
          frontRightModule.getWheelSpeedMeters());
      SmartDashboard.putNumber("Drive: RL Velocity",
          rearLeftModule.getWheelSpeedMeters());
      SmartDashboard.putNumber("Drive: RR Velocity",
          rearRightModule.getWheelSpeedMeters());

      // Display Gyro Angle
      SmartDashboard.putNumber("Drive: Gyro Yaw", navx.getYaw());

      dashboardCounter = 0;
    }

    // DriveTrain Dashboard Update
    if (dashboardCounter >= 5) {
      // Display LimeLight Distance to Target

      dashboardCounter = 0;
    }
  }

  public void configDriveMotors() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(DriveConstants.DriveMotors.inverted);
    config.idleMode(DriveConstants.DriveMotors.idleMode);
    config.smartCurrentLimit(DriveConstants.DriveMotors.currenLimit);
    config.closedLoopRampRate(DriveConstants.DriveMotors.closedLoopRampRate);
    config.openLoopRampRate(DriveConstants.DriveMotors.openLoopRampRate);
    config.closedLoop.pidf(DriveConstants.DriveMotors.driveP, DriveConstants.DriveMotors.driveI,
        DriveConstants.DriveMotors.driveD, DriveConstants.DriveMotors.driveF);
    frontLeftDrive.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    frontRightDrive.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rearLeftDrive.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rearRightDrive.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void configTurnMotors() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(DriveConstants.TurnMotors.inverted);
    config.idleMode(DriveConstants.TurnMotors.idleMode);
    config.smartCurrentLimit(DriveConstants.TurnMotors.currenLimit);
    config.closedLoopRampRate(DriveConstants.TurnMotors.closedLoopRampRate);
    config.openLoopRampRate(DriveConstants.TurnMotors.openLoopRampRate);
    frontLeftTurn.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    frontRightTurn.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rearLeftTurn.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rearRightTurn.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void stopDrive() {
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
  }
}
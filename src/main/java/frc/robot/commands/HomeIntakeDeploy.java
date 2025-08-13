// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploy;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeIntakeDeploy extends Command {
  private IntakeDeploy mIntakeDeploy;

  private boolean beginningLimitState;
  private boolean isDone = false;

  /** Creates a new HomeIntakeDeploy. */
  public HomeIntakeDeploy(IntakeDeploy subsystem) {
    mIntakeDeploy = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeDeploy);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beginningLimitState = mIntakeDeploy.getLimitSwitch();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!beginningLimitState) {
      mIntakeDeploy.setIntakeDeployedSpeed(.15);

      if (mIntakeDeploy.getLimitSwitch()) {
        beginningLimitState = true;
      }
    } else {

      mIntakeDeploy.setIntakeDeployedSpeed(-.15);

      if (!mIntakeDeploy.getLimitSwitch()) {
        isDone = true;
        mIntakeDeploy.setIntakeDeployedSpeed(0.0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntakeDeploy.setIntakeDeployedSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}

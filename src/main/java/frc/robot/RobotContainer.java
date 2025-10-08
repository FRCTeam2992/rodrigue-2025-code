// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter.ShooterMode;

public class RobotContainer {
  public final Turret turret;
  public final Shooter shooter;
  public final Intake intake;

  public final CommandXboxController controllerA;

  public RobotContainer() {
    turret = new Turret();
    shooter = new Shooter();
    intake = new Intake();

    controllerA = new CommandXboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    // Powers are clamped. Check TurretConstants.Limits
    controllerA.a().whileTrue(new InstantCommand(() -> turret.setTurretSpeed(0.15)));
    controllerA.a().onFalse(new InstantCommand(() -> turret.stopTurret()));
    controllerA.b().whileTrue(new InstantCommand(() -> turret.setTurretSpeed(-0.15)));
    controllerA.b().onFalse(new InstantCommand(() -> turret.stopTurret()));

    controllerA.leftBumper().onTrue(new InstantCommand(() -> {
      shooter.setMainShooterPower(0.15);
      shooter.setMode(ShooterMode.ManualSpin);
    }));
    controllerA.leftBumper().onFalse(new InstantCommand(() -> {
      shooter.setMainShooterPower(0.0);
    }));
    controllerA.povUp().and(controllerA.leftBumper())
      .onTrue(new InstantCommand(() -> shooter.updateMainShooterPower(0.05)));
    controllerA.povDown().and(controllerA.leftBumper())
      .onTrue(new InstantCommand(() -> shooter.updateMainShooterPower(-0.05)));

    controllerA.rightBumper().onTrue(new InstantCommand(() -> {
      shooter.setSecondaryShooterPower(0.15);
      shooter.setMode(ShooterMode.ManualSpin);
    }));
    controllerA.rightBumper().onFalse(new InstantCommand(() -> {
      shooter.setSecondaryShooterPower(0.0);
    }));
    controllerA.povUp().and(controllerA.rightBumper())
      .onTrue(new InstantCommand(() -> shooter.updateSecondaryShooterPower(0.05)));
    controllerA.povDown().and(controllerA.rightBumper())
      .onTrue(new InstantCommand(() -> shooter.updateSecondaryShooterPower(-0.05)));
    
    controllerA.x().onTrue(new InstantCommand(() -> {
      shooter.setMode(ShooterMode.Shooting);
    }));


    controllerA.leftTrigger(0.5).onTrue(new InstantCommand(() -> {
      intake.setState(0.2, true);
    }));
    controllerA.leftTrigger(0.5).onFalse(new InstantCommand(() -> {
      intake.setState(0.0, false);
    }));

    int increment = 500;

    controllerA.povUp().and(controllerA.leftBumper().negate()).and(controllerA.rightBumper().negate())
      .onTrue(new InstantCommand(() -> shooter.updateMainShooterTargetRPM(increment)));
    controllerA.povDown().and(controllerA.leftBumper().negate()).and(controllerA.rightBumper().negate())
      .onTrue(new InstantCommand(() -> shooter.updateMainShooterTargetRPM(-increment)));
    controllerA.povRight().onTrue(new InstantCommand(() -> shooter.updateSecondaryShooterTargetRPM(increment)));
    controllerA.povLeft().onTrue(new InstantCommand(() -> shooter.updateSecondaryShooterTargetRPM(-increment)));
    controllerA.y().onTrue(new InstantCommand(() -> shooter.setMode(ShooterMode.Stopped)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  public final Turret turret;

  public final CommandXboxController controllerA;

  public RobotContainer() {
    turret = new Turret();

    controllerA = new CommandXboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    // Powers are clamped. Check TurretConstants.Limits
    controllerA.a().whileTrue(new InstantCommand(() -> turret.setTurretSpeed(0.15)));
    controllerA.a().onFalse(new InstantCommand(() -> turret.stopTurret()));
    controllerA.b().whileTrue(new InstantCommand(() -> turret.setTurretSpeed(-0.15)));
    controllerA.b().onFalse(new InstantCommand(() -> turret.stopTurret()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

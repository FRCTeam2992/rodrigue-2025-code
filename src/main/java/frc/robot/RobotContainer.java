// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Funnel;
import frc.robot.commands.DriveSticks;
import frc.robot.constants.DebugConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.triggers.CompetitionControllers;
import frc.robot.triggers.RobotTriggers;
import frc.robot.triggers.TestControllers;

public class RobotContainer {
  public final Turret turret;
  public final Shooter shooter;
  public final Intake intake;
  public final Feeder feeder;
  public final Funnel funnel;
  public final DriveTrain drivetrain;

  public final CommandXboxController controllerA;

  public final TestControllers testControllerMappings;
  public final CompetitionControllers competitionControllerMappings;

  public final RobotTriggers robotTriggers;

  public RobotContainer() {
    turret = new Turret();
    shooter = new Shooter();
    intake = new Intake();
    feeder = new Feeder();
    funnel = new Funnel();
    drivetrain = new DriveTrain();

    controllerA = new CommandXboxController(0);
    drivetrain.setDefaultCommand(new DriveSticks(drivetrain, controllerA));

    robotTriggers = new RobotTriggers(this);
    robotTriggers.setupRobotTriggers();

    testControllerMappings = new TestControllers(this);
    competitionControllerMappings = new CompetitionControllers(this);
    if (DebugConstants.ControlInterface.enableTestControllers) {
      testControllerMappings.setupControllerAMappings();
    } else {
      competitionControllerMappings.setupControllerAMappings();
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

package frc.robot.triggers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder.FeederMode;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;

/**
 * TestControllers class is responsible for setting up controller bindings
 * for testing and tuning.
 */
public class TestControllers {
    RobotContainer robotContainer;

    CommandXboxController controllerA;
    // CompetitionTriggers triggers;

    /**
     * Constructor for CompetitionControllerMappings.
     * 
     * @param robotContainer The RobotContainer instance that contains subsystems
     *                       and controllers.
     */
    public TestControllers(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.controllerA = robotContainer.controllerA;
        // this.triggers = robotContainer.competitionTriggers;
    }

    /**
     * Sets up the mappings for Controller A.
     */
    public void setupControllerAMappings() {
        // Powers are clamped. Check TurretConstants.Limits
        controllerA.a().whileTrue(new InstantCommand(() -> this.robotContainer.turret.setTurretSpeed(0.15)));
        controllerA.a().onFalse(new InstantCommand(() -> this.robotContainer.turret.stopTurret()));
        controllerA.b().whileTrue(new InstantCommand(() -> this.robotContainer.turret.setTurretSpeed(-0.15)));
        controllerA.b().onFalse(new InstantCommand(() -> this.robotContainer.turret.stopTurret()));

        controllerA.leftBumper().onTrue(new InstantCommand(() -> {
            this.robotContainer.shooter.setMainShooterPower(0.15);
            this.robotContainer.shooter.setMode(ShooterMode.ManualSpin);
        }));
        controllerA.leftBumper().onFalse(new InstantCommand(() -> {
            this.robotContainer.shooter.setMainShooterPower(0.0);
        }));
        controllerA.povUp().and(controllerA.leftBumper())
                .onTrue(new InstantCommand(() -> this.robotContainer.shooter.updateMainShooterPower(0.05)));
        controllerA.povDown().and(controllerA.leftBumper())
                .onTrue(new InstantCommand(() -> this.robotContainer.shooter.updateMainShooterPower(-0.05)));

        controllerA.rightBumper().onTrue(new InstantCommand(() -> {
            this.robotContainer.shooter.setSecondaryShooterPower(0.15);
            this.robotContainer.shooter.setMode(ShooterMode.ManualSpin);
        }));
        controllerA.rightBumper().onFalse(new InstantCommand(() -> {
            this.robotContainer.shooter.setSecondaryShooterPower(0.0);
        }));
        controllerA.povUp().and(controllerA.rightBumper())
                .onTrue(new InstantCommand(() -> this.robotContainer.shooter.updateSecondaryShooterPower(0.05)));
        controllerA.povDown().and(controllerA.rightBumper())
                .onTrue(new InstantCommand(() -> this.robotContainer.shooter.updateSecondaryShooterPower(-0.05)));

        controllerA.x().onTrue(new InstantCommand(() -> {
            this.robotContainer.shooter.setMode(ShooterMode.Shooting);
        }));

        controllerA.leftTrigger(0.5).onTrue(new InstantCommand(() -> {
            this.robotContainer.intake.setState(0.2, IntakeMode.ManualSpin);
            this.robotContainer.feeder.setState(0.6, FeederMode.ManualFeed);
        }));
        controllerA.leftTrigger(0.5).onFalse(new InstantCommand(() -> {
            this.robotContainer.intake.setState(0.0, IntakeMode.Stopped);
            this.robotContainer.feeder.setState(0.0, FeederMode.Stopped);

        }));

        int increment = 500;

        controllerA.povUp().and(controllerA.leftBumper().negate()).and(controllerA.rightBumper().negate())
                .onTrue(new InstantCommand(() -> this.robotContainer.shooter.updateMainShooterTargetRPM(increment)));
        controllerA.povDown().and(controllerA.leftBumper().negate()).and(controllerA.rightBumper().negate())
                .onTrue(new InstantCommand(() -> this.robotContainer.shooter.updateMainShooterTargetRPM(-increment)));
        controllerA.povRight().onTrue(
                new InstantCommand(() -> this.robotContainer.shooter.updateSecondaryShooterTargetRPM(increment)));
        controllerA.povLeft().onTrue(
                new InstantCommand(() -> this.robotContainer.shooter.updateSecondaryShooterTargetRPM(-increment)));
        controllerA.y().onTrue(new InstantCommand(() -> this.robotContainer.shooter.setMode(ShooterMode.Stopped)));

        controllerA.start().onTrue(new InstantCommand(() -> this.robotContainer.drivetrain.resetGyro()));
    }
}

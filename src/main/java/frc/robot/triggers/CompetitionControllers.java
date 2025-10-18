package frc.robot.triggers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants.SpeedConstants;
import frc.robot.subsystems.Feeder.FeederMode;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;

/**
 * CompetitionControllers class is responsible for setting up controller
 * bindings
 * for competition play.
 */
public class CompetitionControllers {
    RobotContainer robotContainer;

    CommandXboxController controllerA;
    RobotTriggers triggers;

    /**
     * Constructor for CompetitionControllerMappings.
     * 
     * @param robotContainer The RobotContainer instance that contains subsystems
     *                       and controllers.
     */
    public CompetitionControllers(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.controllerA = robotContainer.controllerA;
        this.triggers = robotContainer.robotTriggers;
    }

    /**
     * Sets up the mappings for Controller A.
     */
    public void setupControllerAMappings() {
        controllerA.a().onTrue(new InstantCommand(() -> {
            // stop intake
            this.robotContainer.intake.setState(0.0, IntakeMode.Stopped);
            this.robotContainer.feeder.setState(0.0, FeederMode.Stopped);
        }));
        controllerA.b().onTrue(new InstantCommand(() -> {
            // stop shooter
            this.robotContainer.shooter.setMode(ShooterMode.Stopped);
        }));
        controllerA.x().onTrue(new InstantCommand(() -> {
            // start auto-intake
        }));
        controllerA.y().onTrue(new InstantCommand(() -> {
            // start shooter
            this.robotContainer.shooter.setMode(ShooterMode.Shooting);
        }));

        controllerA.leftBumper().onTrue(new InstantCommand(() -> {
            // Manual intake dejam
            this.robotContainer.intake.setState(-0.2, IntakeMode.ManualSpin);
            this.robotContainer.feeder.setState(-0.6, FeederMode.ManualFeed);
        }));
        controllerA.leftBumper().onFalse(new InstantCommand(() -> {
            // Stop when manual intake dejam released
            this.robotContainer.intake.setState(0.0, IntakeMode.Stopped);
            this.robotContainer.feeder.setState(0.0, FeederMode.Stopped);
        }));
        controllerA.leftTrigger(0.5).onTrue(new InstantCommand(() -> {
            // Manual intake
            this.robotContainer.intake.setState(0.2, IntakeMode.ManualSpin);
            this.robotContainer.feeder.setState(0.6, FeederMode.ManualFeed);
        }));
        controllerA.leftTrigger(0.5).onFalse(new InstantCommand(() -> {
            // Stop on manual intake release
            this.robotContainer.intake.setState(0.0, IntakeMode.Stopped);
            this.robotContainer.feeder.setState(0.0, FeederMode.Stopped);
        }));
        controllerA.rightTrigger(0.6).onTrue(new InstantCommand(() -> {
            // When shooter not at speed, ensure shooter is started
            this.robotContainer.shooter.setMode(ShooterMode.Shooting);
        }));
        (controllerA.rightTrigger(0.6).and(triggers.readyToShoot)).onTrue(new InstantCommand(() -> {
            // Manually feed ball into shooter when it is already at speed
            this.robotContainer.feeder.setState(0.6, FeederMode.ManualFeed);
        }));
        controllerA.rightTrigger(0.6).onFalse(new InstantCommand(() -> {
            // Stop the feeder when released
            this.robotContainer.feeder.setState(0.0, FeederMode.Stopped);
        }));

        controllerA.povUp().and(controllerA.rightBumper().negate()).onTrue(new InstantCommand(() -> {
            // Hood up
        }));
        controllerA.povUp().and(controllerA.rightBumper().negate()).onFalse(new InstantCommand(() -> {
            // Stop hood on release
        }));
        controllerA.povDown().and(controllerA.rightBumper().negate()).onTrue(new InstantCommand(() -> {
            // Hood down
        }));
        controllerA.povDown().and(controllerA.rightBumper().negate()).onFalse(new InstantCommand(() -> {
            // Stop hood on release
        }));
        controllerA.povRight().and(controllerA.rightBumper().negate()).onTrue(new InstantCommand(() -> {
            // Turret right
            this.robotContainer.turret.setTurretSpeed(0.15);
        }));
        controllerA.povRight().and(controllerA.rightBumper().negate()).onFalse(new InstantCommand(() -> {
            // Stop turret on release
            this.robotContainer.turret.stopTurret();
        }));
        controllerA.povLeft().and(controllerA.rightBumper().negate()).onTrue(new InstantCommand(() -> {
            // Turret left
            this.robotContainer.turret.setTurretSpeed(-0.15);
        }));
        controllerA.povLeft().and(controllerA.rightBumper().negate()).onFalse(new InstantCommand(() -> {
            // Stop turret on release
            this.robotContainer.turret.stopTurret();
        }));

        controllerA.povUp().and(controllerA.rightBumper()).onTrue(new InstantCommand(() -> {
            // Increment main shooter speed
            this.robotContainer.shooter.updateMainShooterTargetRPM(SpeedConstants.Shooting.shooterSpeedIncrementRPM);
        }));
        controllerA.povDown().and(controllerA.rightBumper()).onTrue(new InstantCommand(() -> {
            // Decrement main shooter speed
            this.robotContainer.shooter.updateMainShooterTargetRPM(-SpeedConstants.Shooting.shooterSpeedIncrementRPM);
        }));
        controllerA.povRight().and(controllerA.rightBumper()).onTrue(new InstantCommand(() -> {
            // Increment secondary shooter speed
            this.robotContainer.shooter.updateSecondaryShooterTargetRPM(SpeedConstants.Shooting.shooterSpeedIncrementRPM);
        }));
        controllerA.povLeft().and(controllerA.rightBumper()).onTrue(new InstantCommand(() -> {
            // Decrement secondary shooter speed
            this.robotContainer.shooter.updateSecondaryShooterTargetRPM(-SpeedConstants.Shooting.shooterSpeedIncrementRPM);
        }));

        controllerA.start().onTrue(new InstantCommand(() -> {
            // Reset the gyro for field-orient
            this.robotContainer.drivetrain.resetGyro();
        }));
    }
}

package frc.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

/**
 * CompetitionTriggers class is responsible for setting up triggers and mappings
 * for competition play.
 */
public class RobotTriggers {
    RobotContainer robotContainer;

    public Trigger readyToShoot;

    /**
     * Constructor for RobotTriggers.
     * 
     * @param robotContainer The RobotContainer instance that contains subsystems and controllers.
     */
    public RobotTriggers(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    /**
     * Sets up the triggers and mappings.
     */
    public void setupRobotTriggers() {
        this.readyToShoot = new Trigger(() -> {
            return robotContainer.shooter.readyToShoot();
        });
    }
}
package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class Devices {
    public static CANBus roboRioCANBus = new CANBus("rio");

    /**
     * Enumeration of the types of RoboRio rails that devices can connect to.
     */
    public enum RioRail {
        DigitalIO,
        AnalogIO,
        PWM;
    }

    /**
     * Device addresses for PWM and sensor devices that plug into RoboRio rails.
     * 
     * Each device provides its RioRail type (DigitalIO, AnalogIO, or PWM) and the
     * channel number it is connected to on that rail.
     */
    public enum RioRailDeviceAddress {
        CargoFunnelLowerProximitySensor(RioRail.DigitalIO, 0),
        CargoFunnelUpperProximitySensor(RioRail.DigitalIO, 1),
        ShooterFeedThroughbore(RioRail.DigitalIO, 2);

        public final RioRail rail;
        public final int channel;

        private RioRailDeviceAddress(RioRail rail, int channel) {
            this.rail = rail;
            this.channel = channel;
        }
    }

    /**
     * Device addresses for CAN devices on the robot.
     * 
     * Each device provides its CAN ID and the CAN bus it is on.
     */
    public enum CANDeviceAddress {
        FrontRightSwerveTurn(2, roboRioCANBus),
        FrontRightSwerveCANCoder(2, roboRioCANBus),
        FrontRightSwerveDrive(3, roboRioCANBus),
        FrontLeftSwerveTurn(4, roboRioCANBus),
        FrontLeftSwerveCANCoder(4, roboRioCANBus),
        FrontLeftSwerveDrive(5, roboRioCANBus),
        RearRightSwerveTurn(6, roboRioCANBus),
        RearRightSwerveCANCoder(6, roboRioCANBus),
        RearRightSwerveDrive(7, roboRioCANBus),
        RearLeftSwerveTurn(8, roboRioCANBus),
        RearLeftSwerveCANCoder(8, roboRioCANBus),
        RearLeftSwerveDrive(9, roboRioCANBus),
        ShooterFlywheelLeader(30, roboRioCANBus),
        ShooterFlywheelFollower(31, roboRioCANBus),
        ShooterBackspin(32, roboRioCANBus),
        ShooterHoodMotor(33, roboRioCANBus),
        ShooterHoodCANCoder(33, roboRioCANBus),
        TurretMotor(34, roboRioCANBus),
        TurretCANCoder(34, roboRioCANBus),
        Intake(21, roboRioCANBus),
        CargoFunnel(22, roboRioCANBus),
        ShooterFeed(24, roboRioCANBus);

        public int id;
        public CANBus bus;

        private CANDeviceAddress(int id, CANBus canBus) {
            this.id = id;
            this.bus = canBus;
        }
    }
}

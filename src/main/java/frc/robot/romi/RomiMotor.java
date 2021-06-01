package frc.robot.romi;

import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;

public class RomiMotor {

    /**
     * Constructs a DC motor.
     * Romi Motor Specs: https://www.pololu.com/product/3675/specs
     *
     * @param nominalVoltageVolts     Voltage at which the motor constants were measured.
     * @param stallTorqueNewtonMeters Current draw when stalled.
     * @param stallCurrentAmps        Current draw when stalled.
     * @param freeCurrentAmps         Current draw under no load.
     * @param freeSpeedRadPerSec      Angular velocity under no load.
     * @param numMotors               Number of motors in a gearbox.
     */
    private final static double nominalVoltageVolts = 4.5;
    private final static double stallTorqueNewtonMeters = 0.1765387958333325;
    private final static double stallCurrentAmps = 1.25;
    private final static double freeCurrentAmps = 0.13;
    private final static double freeSpeedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(150);

    public static DCMotor getMotor(int numMotors) {
        return new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, numMotors);
    }
}

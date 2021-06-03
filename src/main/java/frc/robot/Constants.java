/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.romi.RomiMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
//        public static final double kTrackwidthMeters = Units.inchesToMeters(6.25);
        public static final double kTrackwidthMeters = 0.142072613;
//        public static final double kTrackwidthMeters = 0.14;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        // Example values only -- use what's on your physical robot!
        public static final DCMotor kDriveGearbox = RomiMotor.getMotor(1);
        public static final double kDriveGearing = 1.0 / 120.0;

        private static final double kCountsPerRevolution = 1440.0;
        private static final double kWheelDiameterInch = 2.75;

//        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInch);
        public static final double kWheelDiameterMeters = 0.14;
        public static final double kEncoderDistancePerPulse =
                // Encoders are not on the wheel shaft for Falcons, so need to multiply by gear ratio
                (kWheelDiameterMeters * Math.PI) / kCountsPerRevolution;


        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static double kPleft = 7.54;
        public static double kPright = 0;
//        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;

        public static final double ksVolts = 0.291;
        public static final double kvVoltSecondsPerMeter = 6;
//        public static final double kvVoltSecondsPerMeter = 9.5975;
        public static final double kaVoltSecondsSquaredPerMeter = 0.456;

        public static final double kvVoltSecondsPerRadian = 10.4;
        public static final double kaVoltSecondsSquaredPerRadian = 0.456;

//        public static double kP = 0.085;
//        public static double kI = 0;
//        public static double kD = 0;
//
//        public static final double ksVolts = 0.929;
//        public static final double kvVoltSecondsPerMeter = 6.33;
//        public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
//
//        public static final double kvVoltSecondsPerRadian = 6.33;
//        public static final double kaVoltSecondsSquaredPerRadian = 0.0389;

//        public static double kP = 13;
//        public static double kI = 0;
//        public static double kD = 0;
//
//        public static final double ksVolts = 0.38069;
//        public static final double kvVoltSecondsPerMeter = 9.5975;
//        public static final double kaVoltSecondsSquaredPerMeter = 0.60273;
//
//        public static final double kvVoltSecondsPerRadian = 9.5975;
//        public static final double kaVoltSecondsSquaredPerRadian = 0.60273;

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
                LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);
    }
}

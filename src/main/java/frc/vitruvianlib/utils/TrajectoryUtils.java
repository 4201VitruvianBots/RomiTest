package frc.vitruvianlib.utils;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.RomiRamseteCommand;
import frc.robot.commands.RomiRamseteCommandOld;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.RomiDrivetrain;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class TrajectoryUtils {

    public static ArrayList<Pose2d> readCsvTrajectory(String filename) {
        BufferedReader reader;
        String fileLine;
        String[] fields;
        ArrayList<Pose2d> trajectoryPoints = new ArrayList<>();
        String fullpath = "/home/lvuser/deploy/Trajectories/" + filename + ".csv";
        try {
            reader = new BufferedReader(new FileReader(fullpath));
            while ((fileLine = reader.readLine()) != null) {
                fields = fileLine.split(",");
                trajectoryPoints.add(new Pose2d(Units.feetToMeters(Double.parseDouble(fields[0])),
                        Units.feetToMeters(Double.parseDouble(fields[1])),
                        Rotation2d.fromDegrees(Double.parseDouble(fields[2]))));

            }
        } catch (FileNotFoundException e) {
            System.out.println("Error: Could not find file");
            e.printStackTrace();
        } catch (IOException e) {
            System.out.println("Error: Could not read file");
            e.printStackTrace();
        }
        return trajectoryPoints;
    }

    public static RomiRamseteCommand generateRomiRamseteCommand(RomiDrivetrain driveTrain, ArrayList<Pose2d> path, TrajectoryConfig config, FieldSim fieldSim) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(path, config);

        return generateRomiRamseteCommand(driveTrain, trajectory, fieldSim);
    }

    public static RomiRamseteCommand generateRomiRamseteCommand(RomiDrivetrain driveTrain, Trajectory trajectory, FieldSim fieldSim) {

        RomiRamseteCommand ramseteCommand = new RomiRamseteCommand(
                trajectory,
                driveTrain::getPose,
                new RamseteController(),
                driveTrain.getFeedforward(),
                driveTrain.getDriveTrainKinematics(),
                driveTrain::getWheelSpeeds,
                driveTrain.getLeftPIDController(),
                driveTrain.getRightPIDController(),
                driveTrain::setVoltageOutput,
                driveTrain,
                fieldSim
        );
        return ramseteCommand;
    }
}

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.RomiRamseteCommandOld;
import frc.robot.commands.SetOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.RomiDrivetrain;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

public class DriveStraight extends SequentialCommandGroup {
    public DriveStraight(RomiDrivetrain driveTrain, FieldSim fieldSim) {
        double scalingValue = 2.0 / 15.0; //Shrink the field from 15' x 30' to 2' x 4'

        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(6), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(12), Units.inchesToMeters(6), new Rotation2d(Units.degreesToRadians(0))),
        };
        
        Pose2d startPosition = waypoints[0];

        TrajectoryConfig configA = new TrajectoryConfig(0.1, 0.1);
        configA.setKinematics(driveTrain.getDriveTrainKinematics());
        configA.setReversed(false);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(),
                Constants.DriveConstants.kDriveKinematics,
                6));
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new CentripetalAccelerationConstraint(1.5));


        addCommands(new SetOdometry(driveTrain, fieldSim, startPosition));

        var trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypoints.clone()), configA);

        var trajectoryStates = new ArrayList<Pose2d>();
        trajectoryStates.addAll(trajectory.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));

        fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);
        addCommands(new RomiRamseteCommandOld(driveTrain, trajectory,fieldSim));
//        addCommands(TrajectoryUtils.generateRomiRamseteCommand(driveTrain, trajectory, fieldSim));
        addCommands(new InstantCommand(() -> driveTrain.setVoltageOutput(0,0), driveTrain));
    }
}


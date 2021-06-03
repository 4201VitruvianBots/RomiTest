/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetArcadeDrive;
import frc.robot.commands.SetOdometry;
import frc.robot.commands.auto.routines.AutoNavSlalom;
import frc.robot.commands.auto.routines.DriveStraight;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.vitruvianlib.utils.JoystickWrapper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain m_driveTrain = new RomiDrivetrain();

  private final JoystickWrapper m_controller = new JoystickWrapper(0);

  private FieldSim m_fieldSim;

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_driveTrain);

  private ArrayList<Pose2d> robotPoses = new ArrayList<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_fieldSim = new FieldSim(m_driveTrain);
    m_fieldSim.initSim();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_controller.invertRawAxis(1, true);
    m_controller.invertRawAxis(5, true);

    m_driveTrain.setDefaultCommand(new SetArcadeDrive(m_driveTrain, m_fieldSim, () -> m_controller.getRawAxis(1), ()-> m_controller.getRawAxis(2)));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
//    return new AutoNavSlalom(m_driveTrain, m_fieldSim);
    return new DriveStraight(m_driveTrain, m_fieldSim);
  }

  public void periodic() {
    m_fieldSim.simulationPeriodic();
  }

  public void autonomousInit() {
//    robotPoses.clear();
  }

  public void autonomousPeriodic() {
//    plotRobotPose();
  }

  public void teleopInit() {
    robotPoses.clear();
    double scalingValue = 2.0 / 15.0; //Shrink the field from 15' x 30' to 2' x 4'

    Pose2d[] waypoints = {
            new Pose2d(scalingValue * Units.inchesToMeters(40),  scalingValue * Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
            new Pose2d(scalingValue * Units.inchesToMeters(120), scalingValue * Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
            new Pose2d(scalingValue * Units.inchesToMeters(240), scalingValue * Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
            new Pose2d(scalingValue * Units.inchesToMeters(300), scalingValue * Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
            new Pose2d(scalingValue * Units.inchesToMeters(330), scalingValue * Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
            new Pose2d(scalingValue * Units.inchesToMeters(300), scalingValue * Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(scalingValue * Units.inchesToMeters(240), scalingValue * Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(scalingValue * Units.inchesToMeters(120), scalingValue * Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(scalingValue * Units.inchesToMeters(40),  scalingValue * Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180)))
    };

    Pose2d startPosition = waypoints[0];

    TrajectoryConfig configA = new TrajectoryConfig(0.6, 0.4);

    var trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypoints.clone()), configA);

    var trajectoryStates = new ArrayList<Pose2d>();
    trajectoryStates.addAll(trajectory.getStates().stream()
            .map(state -> state.poseMeters)
            .collect(Collectors.toList()));

    m_fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);


//    var startPose = new Pose2d(2.0 / 15.0 * Units.inchesToMeters(40),  2.0 / 15.0 * Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0)));
    m_driveTrain.resetOdometry(startPosition);
  }
  public void teleopPeriodic() {
    plotRobotPose();
  }

  Pose2d lastPose;
  public void plotRobotPose() {
    if(robotPoses.size() > 80) {
      robotPoses.remove(0);
      m_fieldSim.getField2d().getObject("ActualPath").setPoses(new Pose2d());
    }

    robotPoses.add(m_driveTrain.getPose());
    m_fieldSim.getField2d().getObject("ActualPath").setPoses(robotPoses);
  }
}

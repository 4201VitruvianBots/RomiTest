package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.RomiDrivetrain;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/*  The point of this class was to do two things:
    1) To simplify the construction of a Ramsete Command to follow a path.
    2) To ensure that the command actually ends once you are at your target

    The second point was due to not being sure how the RamseteCommand would behave since it was just introduced in 2020.
    May revisit the need to specify our own end conditions in the future.
 */
public class RomiRamseteCommandOld extends CommandBase {
    private Trajectory m_trajectory;
    private RomiDrivetrain m_driveTrain;
    private FieldSim m_fieldSim;
    private Timer m_timer;
    private double m_duration;
    private RamseteController m_ramseteController = new RamseteController(2.1, 0.8);

    private ArrayList<Pose2d> m_robotPose = new ArrayList<>();

    public RomiRamseteCommandOld(RomiDrivetrain driveTrain, Trajectory trajectory, FieldSim fieldSim) {
        m_timer = new Timer();
        m_trajectory = trajectory;
        m_driveTrain = driveTrain;
        m_fieldSim = fieldSim;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_robotPose.clear();
        m_duration = m_trajectory.getTotalTimeSeconds();
    }

    @Override
    public void execute() {
        var desiredPose = m_trajectory.sample(m_timer.get());

        var poseChassisSpeeds = new ChassisSpeeds(desiredPose.velocityMetersPerSecond, 0, desiredPose.velocityMetersPerSecond * desiredPose.curvatureRadPerMeter);
        var adjustedChassisSpeeds = m_ramseteController.calculate(m_driveTrain.getPose(), desiredPose);

        var targetSpeeds = m_driveTrain.getDriveTrainKinematics().toWheelSpeeds(poseChassisSpeeds);
        var targetSpeedsRamsete = m_driveTrain.getDriveTrainKinematics().toWheelSpeeds(adjustedChassisSpeeds);

        SmartDashboard.putNumber("Left Target Speed", targetSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Left Ramsete", targetSpeedsRamsete.leftMetersPerSecond);
        SmartDashboard.putNumber("Left Speed", m_driveTrain.getWheelSpeeds().leftMetersPerSecond);

        SmartDashboard.putNumber("Right Target Speed", targetSpeeds.rightMetersPerSecond);
        SmartDashboard.putNumber("Right Ramsete", targetSpeedsRamsete.rightMetersPerSecond);
        SmartDashboard.putNumber("Right Speed", m_driveTrain.getWheelSpeeds().rightMetersPerSecond);

        m_driveTrain.drive(adjustedChassisSpeeds.vxMetersPerSecond, adjustedChassisSpeeds.omegaRadiansPerSecond);
//        m_robotPose.add(m_driveTrain.getPose());

//        m_fieldSim.getField2d().getObject("ActualPath").setPoses(m_robotPose);
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > (m_duration * 1.1);
//        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

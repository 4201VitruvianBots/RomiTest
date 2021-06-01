package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
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
public class RomiRamseteCommand extends RamseteCommand {
    private RomiDrivetrain m_driveTrain;
    private FieldSim m_fieldSim;
    private Trajectory m_trajectory;
    private Supplier<Pose2d> m_pose;
    private ArrayList<Pose2d> m_robotPose = new ArrayList<>();
    private Pose2d m_finalPose;

    Timer m_timer = new Timer();

    private double autoDuration, autoStartTime;

    public RomiRamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController controller, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, PIDController leftController, PIDController rightController, BiConsumer<Double, Double> outputVolts, RomiDrivetrain driveTrain, FieldSim fieldSim) {
        super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts, driveTrain);
        m_driveTrain = driveTrain;
        m_fieldSim = fieldSim;
        m_pose = pose;
        m_trajectory = trajectory;

        int trajectorySize = m_trajectory.getStates().size() - 1;
        m_finalPose = m_trajectory.getStates().get(trajectorySize).poseMeters;
    }

    @Override
    public void initialize() {
        super.initialize();
        autoStartTime = Timer.getFPGATimestamp();
        autoDuration = m_trajectory.getTotalTimeSeconds() + 1;
        m_timer.reset();
        m_timer.start();
        m_robotPose.clear();
    }

    @Override
    public void execute() {
        super.execute();
//        var robotPose = m_trajectory.sample(m_timer.get()).poseMeters;
        m_robotPose.add(m_driveTrain.getPose());

        m_fieldSim.getField2d().getObject("ActualPath").setPoses(m_robotPose);
    }

//    @Override
//    public boolean isFinished() {
//        double deltaX = Units.metersToFeet(Math.abs(m_pose.get().getTranslation().getX() - m_finalPose.getX()));
//        double deltaY = Units.metersToFeet(Math.abs(m_pose.get().getTranslation().getY() - m_finalPose.getY()));
//        double deltaRot = Math.abs(m_pose.get().getRotation().getDegrees() - m_finalPose.getRotation().getDegrees());
//        boolean isFinished = ((deltaX < .25) && (deltaY < .25) && (deltaRot < 4));
//        SmartDashboardTab.putNumber("DriveTrain", "Ramsete Delta X", deltaX);
//        SmartDashboardTab.putNumber("DriveTrain", "Ramsete Delta Y", deltaY);
//        SmartDashboardTab.putNumber("DriveTrain", "Ramsete Delta Rot", deltaRot);
//        SmartDashboardTab.putBoolean("DriveTrain", "Ramsete Command Finished", isFinished);
//        return isFinished;
//    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}

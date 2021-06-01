/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.RomiDrivetrain;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SetArcadeDrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final RomiDrivetrain m_driveTrain;
    private final FieldSim m_fieldSim;
    private final DoubleSupplier m_throttle, m_turn;

    private ArrayList<Pose2d> m_robotPose = new ArrayList<>();

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SetArcadeDrive(RomiDrivetrain driveTrain, FieldSim fieldSim, DoubleSupplier throttle, DoubleSupplier turn) {
        m_driveTrain = driveTrain;
        m_fieldSim = fieldSim;
        m_throttle = throttle;
        m_turn = turn;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double joystickY = (Math.abs(m_throttle.getAsDouble()) > 0.1) ? m_throttle.getAsDouble() : 0;
        double joystickX = (Math.abs(m_turn.getAsDouble()) > 0.1) ? m_turn.getAsDouble() : 0;

        m_driveTrain.arcadeDrive(joystickY, joystickX);
//        m_robotPose.add(m_driveTrain.getPose());
//
//        m_fieldSim.getField2d().getObject("ActualPath").setPoses(m_robotPose);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

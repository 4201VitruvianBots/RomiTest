/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.romi.RomiGyro;

public class RomiDrivetrain extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  private double m_leftOutput, m_rightOutput;

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  RomiGyro m_gyro = new RomiGyro();

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Units.inchesToMeters(6.25)));
  DifferentialDriveOdometry odometry;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter);

  private PIDController m_leftPIDController = new PIDController(Constants.DriveConstants.kPleft, 0, 0);
  private PIDController m_rightPIDController = new PIDController(Constants.DriveConstants.kPright, 0, 0);

  public DifferentialDrivetrainSim m_drivetrainSimulator;
  public SimDouble m_gyroSim;
  /**
   * Creates a new RomiDrivetrain.
   */
  public RomiDrivetrain() {
    // DifferentialDrive defaults to having the right side flipped
    // We don't need to do this because the Romi has accounted for this
    // in firmware/hardware
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    m_diffDrive.setRightSideInverted(false);
    m_diffDrive.setSafetyEnabled(false);
    resetEncoders();

    m_drivetrainSimulator = new DifferentialDrivetrainSim(
            Constants.DriveConstants.kDrivetrainPlant,
            Constants.DriveConstants.kDriveGearbox,
            Constants.DriveConstants.kDriveGearing,
            Constants.DriveConstants.kTrackwidthMeters,
            Constants.DriveConstants.kWheelDiameterMeters / 2.0,
            null);
//    m_gyroSim =
    m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
  }

  public void arcadeDrive(double throttle, double turn) {
    turn *= 1.75;
    double leftPWM = throttle + turn;
    double rightPWM = throttle - turn;


//    // Normalization
//    double magnitude = Math.max(Math.abs(leftPWM), Math.abs(rightPWM));
//    if(magnitude > 1.0) {
//      leftPWM *= 1.0 / magnitude;
//      rightPWM *= 1.0 / magnitude;
//    }

    setVoltageOutput(leftPWM * 1.5, rightPWM * 1.5);
  }

  public void drive(double xSpeed, double rotSpeed) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rotSpeed));
    setSpeeds(wheelSpeeds);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    var leftFeedforward = feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    var rightFeedforward = feedforward.calculate(wheelSpeeds.rightMetersPerSecond);

    var leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), wheelSpeeds.leftMetersPerSecond);
    var rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), wheelSpeeds.rightMetersPerSecond);

    setVoltageOutput(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
    m_diffDrive.feed();
  }

  public void setVoltageOutput(double leftVoltage, double rightVoltage) {
    m_leftOutput = leftVoltage;
    m_rightOutput = rightVoltage;
    m_leftMotor.setVoltage(leftVoltage);
    m_rightMotor.setVoltage(-rightVoltage);
  }


  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, m_gyro.getRotation2d());
    resetEncoders();
    resetGyro();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public DifferentialDriveKinematics getDriveTrainKinematics() {
    return kinematics;
  }

  public PIDController getLeftPIDController() {
    return m_leftPIDController;
  }

  public PIDController getRightPIDController() {
    return m_rightPIDController;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()),
            getLeftDistanceMeters(),
            getRightDistanceMeters());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
//    double leftInput = Math.abs(m_leftOutput) > 12.0 ? 12.0 * Math.signum(m_leftOutput) : m_leftOutput;
//    double rightInput = Math.abs(m_rightOutput) > 12.0 ? 12.0 * Math.signum(m_rightOutput) : m_rightOutput;
//    m_drivetrainSimulator.setInputs(leftInput, rightInput);
//    m_drivetrainSimulator.update(0.020);
//
//    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
//    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
//    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
//    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
//    m_gyro.setSimAngleZ(-m_drivetrainSimulator.getHeading().getDegrees());
//    System.out.println(-m_drivetrainSimulator.getHeading().getDegrees());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MecConstants;
import frc.robot.subsystems.MecanumSubsystem;

public class CustomMecanumTrajectoryFollower extends CommandBase {
  private final Timer m_timer = new Timer();
  private final PathPlannerTrajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SimpleMotorFeedforward m_feedforward;
  private final MecanumDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Supplier<Rotation2d> m_desiredRotation;
  private final double m_maxWheelVelocityMetersPerSecond;
  private final PIDController m_frontLeftController;
  private final PIDController m_rearLeftController;
  private final PIDController m_frontRightController;
  private final PIDController m_rearRightController;
  private final Supplier<MecanumDriveWheelSpeeds> m_currentWheelSpeeds;
  private final Consumer<MecanumDriveMotorVoltages> m_outputDriveVoltages;
  private final boolean usingCustomRotationInput;
  private MecanumDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;


  //TODO add note that this does NOT stop the robot at finish
  //Uses PathPlannerState for rotation tracking
  /** Creates a new MecanumTrajectoryFollower. */
  public CustomMecanumTrajectoryFollower(PathPlannerTrajectory trajectory, MecanumSubsystem driveSubsystem) {
    m_trajectory = trajectory;
    m_pose = driveSubsystem::getPose; //Supplier to get the current Pose2d of the robot
    m_feedforward = MecConstants.mecFeedforward;
    m_kinematics = MecConstants.mecKinematics;
    m_controller =
        new HolonomicDriveController(
            new PIDController(MecConstants.xP, MecConstants.xI, MecConstants.xD),
            new PIDController(MecConstants.yP, MecConstants.yI, MecConstants.yD),
            new ProfiledPIDController(
              MecConstants.rotationP, 
              MecConstants.rotationI, 
              MecConstants.rotationD, 
              new TrapezoidProfile.Constraints(
                MecConstants.rotationMaxVel, 
                MecConstants.rotationMaxAcc)));
    m_maxWheelVelocityMetersPerSecond = MecConstants.maxWheelVelocityMetersPerSecond;
    m_desiredRotation = null;
    m_frontLeftController = new PIDController(MecConstants.wheelP, MecConstants.wheelI, MecConstants.wheelD);
    m_rearLeftController = new PIDController(MecConstants.wheelP, MecConstants.wheelI, MecConstants.wheelD);
    m_frontRightController = new PIDController(MecConstants.wheelP, MecConstants.wheelI, MecConstants.wheelD);
    m_rearRightController = new PIDController(MecConstants.wheelP, MecConstants.wheelI, MecConstants.wheelD);
    m_currentWheelSpeeds = driveSubsystem::getCurrentWheelSpeeds; //Supplier to get the current MecanumWheelSpeeds
    m_outputDriveVoltages = driveSubsystem::setDriveMotorsVolts; //Consumer that will give a MecanumDriveMotorVoltages containing the volts to set each motor to
    usingCustomRotationInput = false;

    //TODO add your specific subsystem type
    addRequirements(driveSubsystem);
  }

  //Uses custom rotation supplier for rotation target
  public CustomMecanumTrajectoryFollower(PathPlannerTrajectory trajectory, Supplier<Rotation2d> desiredRotation, MecanumSubsystem driveSubsystem) {
    m_trajectory = trajectory;
    m_pose = driveSubsystem::getPose; //Supplier to get the current Pose2d of the robot
    m_feedforward = MecConstants.mecFeedforward;
    m_kinematics = MecConstants.mecKinematics;
    m_controller =
        new HolonomicDriveController(
            new PIDController(MecConstants.xP, MecConstants.xI, MecConstants.xD),
            new PIDController(MecConstants.yP, MecConstants.yI, MecConstants.yD),
            new ProfiledPIDController(
              MecConstants.rotationP, 
              MecConstants.rotationI, 
              MecConstants.rotationD, 
              new TrapezoidProfile.Constraints(
                MecConstants.rotationMaxVel, 
                MecConstants.rotationMaxAcc)));
    m_maxWheelVelocityMetersPerSecond = MecConstants.maxWheelVelocityMetersPerSecond;
    m_desiredRotation = desiredRotation;
    m_frontLeftController = new PIDController(MecConstants.wheelP, MecConstants.wheelI, MecConstants.wheelD);
    m_rearLeftController = new PIDController(MecConstants.wheelP, MecConstants.wheelI, MecConstants.wheelD);
    m_frontRightController = new PIDController(MecConstants.wheelP, MecConstants.wheelI, MecConstants.wheelD);
    m_rearRightController = new PIDController(MecConstants.wheelP, MecConstants.wheelI, MecConstants.wheelD);
    m_currentWheelSpeeds = driveSubsystem::getCurrentWheelSpeeds; //Supplier to get the current MecanumWheelSpeeds
    m_outputDriveVoltages = driveSubsystem::setDriveMotorsVolts; //Consumer that will give a MecanumDriveMotorVoltages containing the volts to set each motor to
    usingCustomRotationInput = true;

    addRequirements(driveSubsystem);
  }

  

  // Called when the command is initially scheduled.
  public void initialize() {
    var initialState = m_trajectory.sample(0);

    var initialXVelocity =
        initialState.velocityMetersPerSecond * initialState.poseMeters.getRotation().getCos();
    var initialYVelocity =
        initialState.velocityMetersPerSecond * initialState.poseMeters.getRotation().getSin();

    m_prevSpeeds =
        m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialXVelocity, initialYVelocity, 0.0));

    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    var desiredState = (PathPlannerState) m_trajectory.sample(curTime);
    ChassisSpeeds targetChassisSpeeds;
    if (usingCustomRotationInput) {
        targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
    } else {
        targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);
    }

    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(targetChassisSpeeds);

    targetWheelSpeeds.desaturate(m_maxWheelVelocityMetersPerSecond);

    var frontLeftSpeedSetpoint = targetWheelSpeeds.frontLeftMetersPerSecond;
    var rearLeftSpeedSetpoint = targetWheelSpeeds.rearLeftMetersPerSecond;
    var frontRightSpeedSetpoint = targetWheelSpeeds.frontRightMetersPerSecond;
    var rearRightSpeedSetpoint = targetWheelSpeeds.rearRightMetersPerSecond;

    double frontLeftOutput;
    double rearLeftOutput;
    double frontRightOutput;
    double rearRightOutput;

    final double frontLeftFeedforward =
        m_feedforward.calculate(
            frontLeftSpeedSetpoint,
            (frontLeftSpeedSetpoint - m_prevSpeeds.frontLeftMetersPerSecond) / dt);
    final double rearLeftFeedforward =
        m_feedforward.calculate(
            rearLeftSpeedSetpoint,
            (rearLeftSpeedSetpoint - m_prevSpeeds.rearLeftMetersPerSecond) / dt);
    final double frontRightFeedforward =
        m_feedforward.calculate(
            frontRightSpeedSetpoint,
            (frontRightSpeedSetpoint - m_prevSpeeds.frontRightMetersPerSecond) / dt);
    final double rearRightFeedforward =
        m_feedforward.calculate(
            rearRightSpeedSetpoint,
            (rearRightSpeedSetpoint - m_prevSpeeds.rearRightMetersPerSecond) / dt);

    frontLeftOutput =
        frontLeftFeedforward
            + m_frontLeftController.calculate(
                m_currentWheelSpeeds.get().frontLeftMetersPerSecond, frontLeftSpeedSetpoint);
    rearLeftOutput =
        rearLeftFeedforward
            + m_rearLeftController.calculate(
                m_currentWheelSpeeds.get().rearLeftMetersPerSecond, rearLeftSpeedSetpoint);
    frontRightOutput =
        frontRightFeedforward
            + m_frontRightController.calculate(
                m_currentWheelSpeeds.get().frontRightMetersPerSecond, frontRightSpeedSetpoint);
    rearRightOutput =
        rearRightFeedforward
            + m_rearRightController.calculate(
                m_currentWheelSpeeds.get().rearRightMetersPerSecond, rearRightSpeedSetpoint);

    m_outputDriveVoltages.accept(
        new MecanumDriveMotorVoltages(
            frontLeftOutput, frontRightOutput, rearLeftOutput, rearRightOutput));
    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }

}

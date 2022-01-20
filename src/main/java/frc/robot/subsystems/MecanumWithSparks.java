// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Paths;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MecConstants;
import frc.robot.commands.CustomMecanumTrajectoryFollowerSparks;

public class MecanumWithSparks extends SubsystemBase {

  //4 Motors
  Spark fLeftSpark, fRightSpark, rLeftSpark, rRightSpark;

  //4 encoders
  Encoder fLeftEncoder, fRightEncoder, rLeftEncoder, rRightEncoder;

  //1 gyro
  AHRS navX;

  //MechanumDrive
  MecanumDrive drive;

  //Field and odometry
  Field2d field2d;
  MecanumDriveOdometry odometry;

  /** Creates a new MechanumSubsystem. */
  public MecanumWithSparks() {
    //Instantiate Motors and drive
    fLeftSpark = new Spark(MecConstants.fLeftID);
    fRightSpark = new Spark(MecConstants.fRightID);
    rLeftSpark = new Spark(MecConstants.rLeftID);
    rRightSpark = new Spark(MecConstants.rRightID);

    drive = new MecanumDrive(fLeftSpark, rLeftSpark, fRightSpark, rRightSpark);
    drive.setDeadband(MecConstants.deadband);

    //Instantiate Sensors
    fLeftEncoder = new Encoder(MecConstants.fLeftEncoderA, MecConstants.fLeftEncoderB);
    fRightEncoder = new Encoder(MecConstants.fRightEncoderA, MecConstants.fRightEncoderB);
    rLeftEncoder = new Encoder(MecConstants.rLeftEncoderA, MecConstants.rLeftEncoderB);
    rRightEncoder = new Encoder(MecConstants.rRightEncoderA, MecConstants.rRightEncoderB);

    fRightEncoder.setReverseDirection(MecConstants.fRightEncoderReversed);
    fLeftEncoder.setReverseDirection(MecConstants.fLeftEncoderReversed);
    rRightEncoder.setReverseDirection(MecConstants.rRightEncoderReversed);
    rLeftEncoder.setReverseDirection(MecConstants.rLeftEncoderReversed);

    //TODO set distance per pulse
    fLeftEncoder.setDistancePerPulse(MecConstants.distancePerPulseBore);
    fRightEncoder.setDistancePerPulse(MecConstants.distancePerPulseBore);
    rLeftEncoder.setDistancePerPulse(MecConstants.distancePerPulseBore);
    rRightEncoder.setDistancePerPulse(MecConstants.distancePerPulseBore);

    navX = new AHRS(SPI.Port.kMXP);
    navX.reset();

    //Odometry Tracking
    odometry = new MecanumDriveOdometry(MecConstants.mecKinematics, Rotation2d.fromDegrees(getHeading()));
    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);

    //Simulation code
    //Hmm i dont know how to simulate this

  }


  //Basic mecanum drive methods

  //Drive cartesian
  //Drive field centric

  public void stopDrive() {
    drive.stopMotor();
  }

  public void setDriveMotorsVolts(MecanumDriveMotorVoltages voltages) {
    fLeftSpark.setVoltage(voltages.frontLeftVoltage);
    fRightSpark.setVoltage(voltages.frontRightVoltage);
    rLeftSpark.setVoltage(voltages.rearLeftVoltage);
    rRightSpark.setVoltage(voltages.rearRightVoltage);
    drive.feed();
  }


  //Sensor methods

  public double getHeading() {
    return navX.getAngle();
  }

  public double getTurnRate() {
    return navX.getRate();
  }
  
  public void resetGyro() {
    navX.reset();
  }

  public void calibrateGyro() {
    navX.calibrate();
  }

  public void resetEncoders() {
    fLeftEncoder.reset();
    fRightEncoder.reset();
    rRightEncoder.reset();
    rLeftEncoder.reset();
  }


  //Trajectory following methods

  public void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), getCurrentWheelSpeeds());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void setPose(Pose2d startingPose) {
    resetEncoders();
    //TODO might possibly need a -getAngle() call (however, we don't seem to if we have NavX)
    odometry.resetPosition(startingPose, Rotation2d.fromDegrees(getHeading()));
  }

  //TODO Must be in m/s
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      fLeftEncoder.getRate(),
      fRightEncoder.getRate(), 
      rLeftEncoder.getRate(), 
      rRightEncoder.getRate());
  }


  //Trajectory command generations methods

    /**
   * Takes a given JSON name and converts it to a WPILib Trajectory object. This method assumes that
   * the given String is the name of a PathWeaver Path and will automatically add the .wpilib.json suffix
   * and knows where the PathWeaver JSONs are stored. 
   * For this project the JSONs should be stored in src\main\java\frc\robot\output 
   * 
   * @param pathWeaverJSONName The name of the PathWeaver JSON that you would like to load the trajectory from.
   *                           The .wpilib.json is added automatically, so the name should only be the 
   *                           pathName part from this example: output\<i><b>pathName</b></i>.wpilib.json
   * @return the Trajectory loaded from the given PathWeaver JSON
   */
  public Trajectory loadTrajectoryFromPWJSON(String pathWeaverJSONName) {
    try {
      var filePath = Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", pathWeaverJSONName + ".wpilib.json"));
      return TrajectoryUtil.fromPathweaverJson(filePath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + pathWeaverJSONName, ex.getStackTrace());
      return new Trajectory();
    }
  }

  public static PathPlannerTrajectory loadPathPlannerTrajectory(String PathName, double maxVel, double maxAccel, Boolean reversed) {
    return PathPlanner.loadPath(PathName, maxVel, maxAccel, reversed);
  }

  public static PathPlannerTrajectory PathPlannerTrajectory(String PathName, double maxVel, double maxAccel) {
    return PathPlanner.loadPath(PathName, maxVel, maxAccel, false);
  }

    /**
   * Takes a given trajectory and creates a CustomRamseteCommand from the trajectory automatically. If
   * initPose is set to true, it will also add a Command that will set the pose of the robot (set the 
   * driveOdometry's pose2D) to the starting pose of the given trajectory. Note: it only sets the pose
   * when this command is executed, not when it is created.  
   * 
   * @param trajectory Trajectory to be used to create the CustomRamseteCommand
   * @param initPose Whether the starting pose of the Trajectory should be used to reset the pose 
   *                 of the drivetrain
   * @return The CustomRamseteCommand/Command created
   */
  public Command createCommandFromPlannerTrajectory(PathPlannerTrajectory trajectory, boolean isInitPose, boolean stopAtEnd) {
    if (isInitPose && stopAtEnd) {
      return new InstantCommand(() -> this.setPose(trajectory.getInitialPose()))
        .andThen(new CustomMecanumTrajectoryFollowerSparks(trajectory, this))
        .andThen(new InstantCommand(this::stopDrive));
    } else if (isInitPose) {
      return new InstantCommand(() -> this.setPose(trajectory.getInitialPose()))
      .andThen(new CustomMecanumTrajectoryFollowerSparks(trajectory, this));
    } else if (stopAtEnd) {
      return new CustomMecanumTrajectoryFollowerSparks(trajectory, this)
        .andThen(new InstantCommand(this::stopDrive));
    } else {
    return new CustomMecanumTrajectoryFollowerSparks(trajectory, this);
    }
  }

  /**
   * Take  a given trajectory and creates a CustomRamseteCommand from the trajectory automatically. 
   * 
   * @param trajectory Trajectory to be used to create the CustomRamseteCommand
   * @return The CustomRamseteCommand/Command created
   */
  public Command createCommandFromPlannerTrajectory(PathPlannerTrajectory trajectory) {
      return new CustomMecanumTrajectoryFollowerSparks(trajectory, this);
  }

  @Override
  public void periodic() {
    updateOdometry();
    field2d.setRobotPose(odometry.getPoseMeters());
    //TODO NOTE THIS SHOULD BE REMOVED LATER
    drive.feed();
    SmartDashboard.putNumber("Front Left Encoder Dis", fLeftEncoder.getDistance());
    SmartDashboard.putNumber("Front Right Encoder Dis", fRightEncoder.getDistance());
    SmartDashboard.putNumber("Rear Left Encoder Dis", rLeftEncoder.getDistance());
    SmartDashboard.putNumber("Rear Right Encoder Dis", rRightEncoder.getDistance());
    SmartDashboard.putNumber("Front Left Encoder Vel", fLeftEncoder.getRate());
    SmartDashboard.putNumber("Front Right Encoder Vel", fRightEncoder.getRate());
    SmartDashboard.putNumber("Rear Left Encoder Vel", rLeftEncoder.getRate());
    SmartDashboard.putNumber("Rear Right Encoder Vel", rRightEncoder.getRate());
  }
}

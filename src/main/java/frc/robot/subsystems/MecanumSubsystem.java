// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.util.RootNameLookup;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MecConstants;

public class MecanumSubsystem extends SubsystemBase {

  //4 Motors
  CANSparkMax fLeftSpark, fRightSpark, rLeftSpark, rRightSpark;

  //4 encoders
  RelativeEncoder fLeftEncoder, fRightEncoder, rLeftEncoder, rRightEncoder;

  //1 gyro
  AHRS navX;

  //MechanumDrive
  MecanumDrive drive;

  //Field and odometry
  Field2d field2d;
  MecanumDriveOdometry odometry;

  /** Creates a new MechanumSubsystem. */
  public MecanumSubsystem() {
    //Instantiate Motors and drive
    fLeftSpark = new CANSparkMax(MecConstants.fLeftID, MotorType.kBrushless);
    fRightSpark = new CANSparkMax(MecConstants.fRightID, MotorType.kBrushless);
    rLeftSpark = new CANSparkMax(MecConstants.rLeftID, MotorType.kBrushless);
    rRightSpark = new CANSparkMax(MecConstants.rRightID, MotorType.kBrushless);

    drive = new MecanumDrive(fLeftSpark, rLeftSpark, fRightSpark, rRightSpark);

    //Instantiate Sensors
    fLeftEncoder = fLeftSpark.getEncoder();
    fRightEncoder = fRightSpark.getEncoder();
    rLeftEncoder = rLeftSpark.getEncoder();
    rRightEncoder = rRightSpark.getEncoder();

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
  //Stop Drive

  public void setDriveFromVolts(MecanumDriveMotorVoltages voltages) {
    fLeftSpark.setVoltage(voltages.frontLeftVoltage);
    fRightSpark.setVoltage(voltages.frontRightVoltage);
    rLeftSpark.setVoltage(voltages.rearLeftVoltage);
    rRightSpark.setVoltage(voltages.rearRightVoltage);
  }


  //Sensor methods

  public double getHeading() {
    return navX.getAngle();
  }
  //Not sure if I need a seperate method for each encoder as each encoder shouldnt be used outside this class
  
  public void resetGyro() {
    navX.reset();
  }

  public void calibrateGyro() {
    navX.calibrate();
  }

  public void resetEncoders() {
    fLeftEncoder.setPosition(0);
    fRightEncoder.setPosition(0);
    rRightEncoder.setPosition(0);
    rLeftEncoder.setPosition(0);
  }


  //Trajectory following methods

  public void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), getWheelSpeeds());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void setPose(Pose2d startingPose) {
    resetEncoders();
    //TODO might possibly need a -getAngle() call (however, we don't seem to if we have NavX)
    odometry.resetPosition(startingPose, Rotation2d.fromDegrees(getHeading()));
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      fLeftEncoder.getVelocity(),
      fRightEncoder.getVelocity(), 
      rLeftEncoder.getVelocity(), 
      rRightEncoder.getVelocity());
  }

  
  //Trajectory command generations methods

  //

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

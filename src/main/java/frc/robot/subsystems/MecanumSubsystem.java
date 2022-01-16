// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MecanumSubsystem extends SubsystemBase {

  //4 Motors
  CANSparkMax fLeftSpark, fRightSpark, rLeftSpark, rRightSpark;

  //4 encoders
  RelativeEncoder fLeftEncoder, fRighEncoder, rLeftEncoder, rRighEncoder;

  //1 gyro
  AHRS navX;

  //MechanumDrive
  MecanumDrive drive;

  //Field and odometry
  Field2d field2d;
  MecanumDriveOdometry odometry;

  /** Creates a new MechanumSubsystem. */
  public MecanumSubsystem() {
    //Instantiate Motors
    fLeftSpark = new CANSparkMax(2, MotorType.kBrushless);
    fRightSpark = new CANSparkMax(3, MotorType.kBrushless);
    rLeftSpark = new CANSparkMax(4, MotorType.kBrushless);
    rRightSpark = new CANSparkMax(5, MotorType.kBrushless);

    //Hmm i dont know how to simulate this

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

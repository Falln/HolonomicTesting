// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.ConstantsPW;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Simple command that simplifies the RamseteCommand's complex constructor
 * down to two arguments: 
 * <p>1. The Trajectory to follow
 * <p>2. The DriveSubsystem
 * <p>Also stops the drivetrain no matter what on completion which RamseteCommand
 * seems to fail to do at times
 */
public class CustomRamseteCommand extends RamseteCommand {

  DriveSubsystem driveSubsystem;

  /** Creates a new CustomRamseteCommand. */
  public CustomRamseteCommand(Trajectory trajectory, DriveSubsystem driveSubsystem) {
    super(
      trajectory,
      driveSubsystem::getPose,
      new RamseteController(ConstantsPW.kRamseteB, ConstantsPW.kRamseteZeta),
      new SimpleMotorFeedforward(ConstantsPW.ksVolts,
                                 ConstantsPW.kvVoltSecondsPerMeter,
                                 ConstantsPW.kaVoltSecondsSquaredPerMeter),
      ConstantsPW.kDriveKinematics,
      driveSubsystem::getWheelSpeeds,
      new PIDController(ConstantsPW.driveP, 0, 0),
      new PIDController(ConstantsPW.driveP, 0, 0),
      driveSubsystem::driveTankVolts,
      driveSubsystem);
      this.driveSubsystem = driveSubsystem;
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    driveSubsystem.stopDrive();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TankDriveCommand extends CommandBase {
  DriveSubsystem driveSubsystem;
  DoubleSupplier leftInput;
  DoubleSupplier rightInput;
  boolean isFinished;
  /** Creates a new TankDrive. */
  public TankDriveCommand(DoubleSupplier leftInput, DoubleSupplier rightInput,DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.leftInput = leftInput;
    this.rightInput = rightInput;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO this is set to driveTankVolts for the simulation, as I cant keep track of volts through driveTank
    driveSubsystem.driveTankVolts(leftInput.getAsDouble()*12, rightInput.getAsDouble()*12);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

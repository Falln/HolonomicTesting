// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MecanumSubsystem;
import frc.robot.subsystems.MecanumWithSparks;


public class RobotContainer {

  //Subsystems
  //private final MecanumSubsystem mecanumSubsystem = new MecanumSubsystem();
  private final MecanumWithSparks mecanumWithSparks = new MecanumWithSparks();

  //Controllers and Triggers
  //private final XboxController driverController = new XboxController(0);

  //PathWeaverJSONs
  PathData newPath = new PathData("New Path", false);
  Command newPathCommand;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Silence the "Missing Joystick" warnings
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //NetworkTable things

    //Configure the Button/Trigger bindings
    configureButtonBindings();

    //Set the default commands of subsystems
    mecanumWithSparks.setDefaultCommand(
      new InstantCommand(mecanumWithSparks::stopDrive, mecanumWithSparks).perpetually());

    //Load all paths
    loadPathPlannerTrajectories(newPath);
    newPathCommand = mecanumWithSparks.createCommandFromPlannerTrajectory(
    newPath.trajectory,
    true, true);
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      
  }

  /**
   * Takes the given PathDatas, generates the trajectory associated with the
   * JSONName they have, and then sets the PathData's trajectory to that trajectory
   * @param pathWeaverData PathData class to load trajectories to
   */
  private void loadPathPlannerTrajectories(PathData... pathData) {
    for (PathData pData:pathData) {
      pData.trajectory = MecanumSubsystem.loadPathPlannerTrajectory(
        pData.PathName,
        pData.maxVel,
        pData.maxAccel,
        pData.reversed);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return newPathCommand;

  }

  /**
   * This class simply allows us to store the PathPlanner name and the 
   * trajectory associated with it in the same place (and possibly the 
   * command created from it too - not used currently). It also allow you
   * to specify specifc max velocity and accelerations if needed.
   */
  public class PathData {
    public String PathName;
    public PathPlannerTrajectory trajectory;
    public boolean reversed;
    public double maxVel = MecConstants.driveMaxVel;
    public double maxAccel = MecConstants.driveMaxAcc;
    
    public PathData(String pathWeaverJSON, boolean reversed) 
      {PathName = pathWeaverJSON; this.reversed = reversed;}

    public PathData(String pathWeaverJSON, boolean reversed, 
                    double maxVel, double maxAccel) 
      {PathName = pathWeaverJSON; this.reversed = reversed;
       this.maxVel = maxVel; this.maxAccel = maxAccel;}
  }


}

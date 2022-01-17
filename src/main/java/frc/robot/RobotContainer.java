// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.html.HTMLDocument.RunElement;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;


public class RobotContainer {

  /**
   * This class simply allows us to store the PathWeaver JSON name and the 
   * trajectory associated with it in the same place (and possibly the 
   * command created from it too - not used currently) 
   */
  class PathData {
    public String JSONName;
    public Trajectory trajectory;
    public Command command;
    public PathData(String pathWeaverJSON) {JSONName = pathWeaverJSON;}
  }

  //Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  //Controllers and Triggers
  private final XboxController driverController = new XboxController(0);

  //PathWeaverJSONs
  PathData testPath1 = new PathData("testPath1");
  PathData testPath2 = new PathData("testPath2");
  PathData newNewPathCopy = new PathData("New New Path Copy");
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Silence the "Missing Joystick" warnings
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //NetworkTable things

    //Configure the Button/Trigger bindings
    configureButtonBindings();

    //Set the default commands of subsystems

    //Load all paths
    loadPathWeaverTrajectories(testPath1, testPath2, newNewPathCopy);
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
  private void loadPathWeaverTrajectories(PathData... pathWeaverData) {
    for (PathData pwData:pathWeaverData) {
      pwData.trajectory = driveSubsystem.loadTrajectoryFromPWJSON((pwData.JSONName));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Generate the path commands

    //Return the combined command
    //return path1Command.andThen(new WaitCommand(2)).andThen(path2Command);
    return null;

  }


}

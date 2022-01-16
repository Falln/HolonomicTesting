package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.CustomRamseteCommand;

public class DriveSubsystem extends SubsystemBase {

  CANSparkMax leftSpark;
  CANSparkMax rightSpark;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  AHRS navX;

  DifferentialDrive drive;
  DifferentialDrivetrainSim driveSim;

  //Keeps track of applied volts if the tankDriveVolts method is called.
  //Note this is only accurate for when tankDriveVolts is called, as this is appliedVoltage
  //and not outputVoltage
  double voltsSuppliedLeft = 0;
  double voltsSuppliedRight = 0;

  Field2d field2d;
  DifferentialDriveOdometry odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //Instantiate motor controllers and reverse them as reverse is the true robot front
    leftSpark = new CANSparkMax(4, MotorType.kBrushless);
    rightSpark = new CANSparkMax(5, MotorType.kBrushless);

    //Simulated robots and real robots can have different inversion factors
    if (Robot.isSimulation()) {
      leftSpark.setInverted(true);
      rightSpark.setInverted(true);
    } else {
      leftSpark.setInverted(true);
      rightSpark.setInverted(true);
    }

    //Get the RelativeEncoder object from the sparks 
    leftEncoder = leftSpark.getEncoder();
    rightEncoder = rightSpark.getEncoder();
    //TODO figure out correct conversion factor. NOTE: must be in meters
    leftEncoder.setPositionConversionFactor(Constants.driveEncConversionFactor);
    rightEncoder.setPositionConversionFactor(Constants.driveEncConversionFactor);

    //Instantiate the navX and reset it as we're booting up
    navX = new AHRS(SPI.Port.kMXP);
    navX.reset();

    //Instantiate the odometry object and the field2D
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);

    //Instantiate the drivetrain
    drive = new DifferentialDrive(leftSpark, rightSpark);
    drive.setDeadband(0.07); //Same thing as setting a min input value but a little different

    //Things needed for drive simulation
    //Only create if its a simulation
    if (Robot.isSimulation()) {
      driveSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDoubleNEOPerSide, // 2 CIMs per side.
        KitbotGearing.k10p71,        // 10.71:1
        KitbotWheelSize.kSixInch,     // 6" diameter wheels.
        null                         // No measurement noise.
        );
    }
  }

  /** Set the power of the left and right sides of the drivetrain independently */
  public void driveTank(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }
  
  /** Drive the drivetrain similar to a video game 1st person shooter */
  public void driveArcade(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  /** Sets the drivetrains speed in volts rather than based on a duty-cycle */
  public void driveTankVolts(double leftVolts, double rightVolts) {
    leftSpark.setVoltage(leftVolts);
    rightSpark.setVoltage(-rightVolts);
    voltsSuppliedLeft = leftVolts;
    voltsSuppliedRight = rightVolts;
    drive.feed();
  }

  /** Stop all drive motors */
  public void stopDrive() {
    drive.stopMotor();
    voltsSuppliedLeft = 0;
    voltsSuppliedRight = 0;
  }

  /** Returns the distance in meters the left encoder has traveled */
  public double getLeftEncoderDistance() {
    return leftEncoder.getPosition();
  }

  /** Returns the distance in meters the right encoder has traveled */
  public double getRightEncoderDistance() {
    return rightEncoder.getPosition();
  }

  /** Returns the average distance in meters both encoders have traveled */
  public double getAveEncDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance())/2;
  }

  /** Resets the position of both encoders to 0 */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
    if (Robot.isSimulation()) {
      setSimDoubleFromDeviceData("SPARK MAX [4]", "Position", 0);
      setSimDoubleFromDeviceData("SPARK MAX [5]", "Position", 0);
    }
  }

  /** Returns a DiffDriveWheelSpeeds object based on the left and right encoder velocity in m/s */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  /** Gets the angle from gyro in degrees. Note this is the yaw of the navX */
  public double getAngle() {
    return navX.getAngle();
  }

  /** Resets all values of the gyro */
  public void resetGyro() {
    navX.reset();

    if (Robot.isSimulation()) {
      setSimDoubleFromDeviceData("navX-Sensor[0]", "Yaw", 0);
    }
  }

  /** 
   * Gets the current calculated pose of the robot. This is determined by the driveOdometry that is 
   * updated in the periodic loop
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Updates the drive odometry */
  public void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getAngle()), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  /**
   * Resets the odometry to the specified pose. If this is a simulation, it will also set the pose
   * of the drivetrain simulator to the specified pose.
   * 
   * @param pose The pose to which to set the odometry.
   */
  public void setPose(Pose2d startingPose) {
    resetEncoders();
    //TODO might possibly need a -getAngle() call
    odometry.resetPosition(startingPose, Rotation2d.fromDegrees(getAngle()));
    if (Robot.isSimulation()) {
      driveSim.setPose(startingPose);
    }
  }

  /** Allows the manual setting/resetting of the simulated drive pose */
  public void setSimPose(Pose2d startingPose) {
    driveSim.setPose(startingPose);
  }

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
  public Command createCommandFromTrajectory(Trajectory trajectory, boolean initPose) {
    if (initPose) {
      return new InstantCommand(() -> this.setPose(trajectory.getInitialPose()))
        .andThen(new CustomRamseteCommand(trajectory, this));
    }
    return new CustomRamseteCommand(trajectory, this);
    }

  /**
   * Takes a given trajectory and creates a CustomRamseteCommand from the trajectory automatically. 
   * 
   * @param trajectory Trajectory to be used to create the CustomRamseteCommand
   * @return The CustomRamseteCommand/Command created
   */
  public Command createCommandFromTrajectory(Trajectory trajectory) {
      return new CustomRamseteCommand(trajectory, this);
  }


  /** In progress */
  public PathPlannerTrajectory getPlannerPath(String JSONName, double maxVel, double maxAccel) {
    return PathPlanner.loadPath(JSONName, maxVel, maxAccel);
  }



  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    updateOdometry();
    field2d.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putNumber("Robot x", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Robot y", odometry.getPoseMeters().getTranslation().getY());
  }

  @Override
  public void simulationPeriodic() {
    // driveSim.setInputs(-leftSpark.get() * RobotController.getInputVoltage(),
    // -rightSpark.get() * RobotController.getInputVoltage());
    driveSim.setInputs(voltsSuppliedLeft, voltsSuppliedRight);

    driveSim.update(0.02);

    //update navX and Spark data (as much as needed)
    setSimDoubleFromDeviceData("navX-Sensor[0]", "Yaw", driveSim.getHeading().getDegrees());
    setSimDoubleFromDeviceData("SPARK MAX [4]", "Position", driveSim.getLeftPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [5]", "Position", driveSim.getRightPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [4]", "Applied Output", voltsSuppliedLeft);
    setSimDoubleFromDeviceData("SPARK MAX [5]", "Applied Output", voltsSuppliedRight);
  }
  

  /** 
   * Allows for shorter more direct references of SimDoubles. Takes a SimDevice device name,
   * a key that points to a SimDouble, and a double to set the SimDouble to.
   * 
   * @param deviceName The name of the SimDevice
   * @param keyName The key that the double is associated with'
   * @param value Double that will be set to the SimDouble at the given key
   */
  public void setSimDoubleFromDeviceData(String deviceName, String keyName, double value) {
    SimDouble simDouble = new SimDouble(
      SimDeviceDataJNI.getSimValueHandle(
          SimDeviceDataJNI.getSimDeviceHandle(deviceName),
          keyName));
    simDouble.set(value);
  }
}

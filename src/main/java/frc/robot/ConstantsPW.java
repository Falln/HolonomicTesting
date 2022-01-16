package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public final class ConstantsPW {
 
    //Drivetrain Feedforward and Feedback gains (constants) derived from Robot Characterization
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double driveP = 8.5;

    //Drivetrain kinematics derived from Robot Characterization. Allows conversion from chassisSpeed to wheelSpeed
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    //Known maximums of the drivetrain derived from Robot Characterization
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    //Constants for the Ramsete Followers - in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //Voltage Constraints based on the Robot Characterization data.
    //Drivetrain voltage constraint based on 10V
    public static final DifferentialDriveVoltageConstraint autonDriveVoltageConstraint10V = 
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                ksVolts,
                kvVoltSecondsPerMeter,
                kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            10);

    //Trajectory configs
    //Trajectory config for autonomous
    public static final TrajectoryConfig autonTrajectoryConfig = 
        new TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autonDriveVoltageConstraint10V);
    }
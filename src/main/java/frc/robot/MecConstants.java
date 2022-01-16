package frc.robot;

import java.nio.file.DirectoryNotEmptyException;

import com.fasterxml.jackson.databind.deser.std.StringArrayDeserializer;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

public final class MecConstants {

    //Constants for a mecanum drivetrain

    //Drivetrain PID/F constants
    public static final double xP = 1;
    public static final double xI = 0;
    public static final double xD = 0;

    public static final double yP = 1;
    public static final double yI = 0;
    public static final double yD = 0;
    
    public static final double rotationP = 1;
    public static final double rotationI = 0;
    public static final double rotationD = 0;
    public static final double rotationMaxVel = 10;
    public static final double rotationMaxAcc = 5;

    public static final double wheelP = 1;
    public static final double wheelI = 0;
    public static final double wheelD = 0;

    public static final double ks = 0;
    public static final double kv = 0;
    public static final SimpleMotorFeedforward mecFeedforward = new SimpleMotorFeedforward(ks, kv);

    //Kinematics Constant
    public static final MecanumDriveKinematics mecKinematics = new MecanumDriveKinematics(
        new Translation2d(), //frontLeftWheelMeters
        new Translation2d(), //frontRightWheelMeters
        new Translation2d(), //rearLeftWheelMeters
        new Translation2d()  //rearRightWheelMeters
        );
    public static final double maxWheelVelocityMetersPerSecond = 10;
}
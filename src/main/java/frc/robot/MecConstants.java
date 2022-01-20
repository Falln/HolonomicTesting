package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class MecConstants {

    //Constants for a mecanum drivetrain

    //Ports and data
    public static final int fLeftID = 0;
    public static final int fRightID = 1;
    public static final int rLeftID = 2;
    public static final int rRightID = 3;

    public static final int fLeftEncoderA = 0;
    public static final int fLeftEncoderB = 1;
    public static final int fRightEncoderA = 2;
    public static final int fRightEncoderB = 3;
    public static final int rLeftEncoderA = 4;
    public static final int rLeftEncoderB = 5;
    public static final int rRightEncoderA = 6;
    public static final int rRightEncoderB = 7;

    public static final boolean fLeftEncoderReversed = false;
    public static final boolean fRightEncoderReversed = true;
    public static final boolean rLeftEncoderReversed = false;
    public static final boolean rRightEncoderReversed = true;

    public static final double wheelCircumference = Units.inchesToMeters(6)*Math.PI;
    public static final double distancePerPulseBore = wheelCircumference/2048;

    public static final double deadband = 0.1;

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

    public static final double wheelP = 1;
    public static final double wheelI = 0;
    public static final double wheelD = 0;

    public static final double ks = 0.51208;
    public static final double kv = 2.4627;
    public static final double ka = 0;
    public static final SimpleMotorFeedforward mecFeedforward = new SimpleMotorFeedforward(ks, kv);

    //Kinematics Constants
    // Distance between centers of right and left wheels on robot
    public static final double trackWidth = 0.5;
    // Distance between centers of front and back wheels on robot
    public static final double wheelBase = 0.7;

    public static final MecanumDriveKinematics mecKinematics = new MecanumDriveKinematics(
        new Translation2d(wheelBase/2, trackWidth/2), //frontLeftWheelMeters
        new Translation2d(wheelBase/2, -trackWidth/2), //frontRightWheelMeters
        new Translation2d(-wheelBase/2, trackWidth/2), //rearLeftWheelMeters
        new Translation2d(-wheelBase/2, -trackWidth/2)  //rearRightWheelMeters
        );

    //NOTE THESE BELOW ARE IN FEET PER SECOND 
    public static final double maxWheelVelocityMetersPerSecond = Units.feetToMeters(10);
    public static final double driveMaxVel = Units.feetToMeters(10);
    public static final double driveMaxAcc = Units.feetToMeters(5);
    public static final double rotationMaxVel = Units.feetToMeters(10);
    public static final double rotationMaxAcc = Units.feetToMeters(5);
}
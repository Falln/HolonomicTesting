package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MecConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;


public class IntakeSubsystem extends SubsystemBase {

    CANSparkMax intake;
    RelativeEncoder intakeEncoder;
    Solenoid lifter;

    public IntakeSubsystem() {
        intake = new CANSparkMax(MecConstants.intakeID, MotorType.kBrushless);

        intakeEncoder = intake.getEncoder();

        lifter = new Solenoid(PneumaticsModuleType.REVPH, MecConstants.lifterID);
        
    }

    //running

    //set power
    public void setPower(double power) {
        intake.set(power);
    }
    //set volatage
    public void setVoltage(double Volts) {
        intake.setVoltage(Volts);
        
    }
    //stop
    public void setPower() {
        intake.stopMotor();
    }
    //intakeIn()

    //lifting/solenoid stuff

    //raise lifter
    //lower lifter
    //toggle
   
    //sensors

    //get velocity
    //reset
}

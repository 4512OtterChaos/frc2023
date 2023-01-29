package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    Spark leftMotor = new Spark(5);
    Spark rightMotor = new Spark(4);

    public Intake(){
        rightMotor.setSafetyEnabled(false);
        leftMotor.setSafetyEnabled(false);
        rightMotor.setInverted(true);
    }
 
    public void setSpeed(double speed){
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    // Command factories
    public CommandBase setSpeedC(double speed) {
        return runEnd(()->setSpeed(speed), ()->setSpeed(0));
    }
}

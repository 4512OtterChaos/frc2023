package frc.robot.subsystems.drive.intake;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    Spark leftMotor = new Spark(6);
    Spark rightMotor = new Spark(5);

    public Intake(){
        rightMotor.setSafetyEnabled(false);
        leftMotor.setSafetyEnabled(false);
        rightMotor.setInverted(true);
    }
 
    /**
     * Sets the speed of the intake.
     * @param speed Speed to set the intake to.
     */
    public void setSpeed(double speed){
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    /**
     * Sets the speed of the intake.
     * @param speed Speed to set the intake to.
     * @return The command to set the intake speed.
     */
    public CommandBase setSpeedC(double speed) {
        return runEnd(()->setSpeed(speed), ()->setSpeed(0));
    }
}

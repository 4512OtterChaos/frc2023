package frc.robot.subsystems.intake;

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
     * Sets the voltage of the intake.
     * @param volts Voltage to set the intake to.
     */
    public void setVoltage(double volts){
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    /**
     * Sets the voltage of the intake.
     * @param volts Voltage to set the intake to.
     * @return The command to set the intake voltage.
     */
    public CommandBase setVoltageC(double volts) {
        return runEnd(()->setVoltage(volts), ()->setVoltage(volts));
    }
}

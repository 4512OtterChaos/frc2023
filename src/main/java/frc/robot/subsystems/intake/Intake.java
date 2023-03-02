package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OCConfig;
import frc.robot.util.OCSparkMax;
import io.github.oblarg.oblog.Loggable;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase{
    
    OCSparkMax leftMotor = new OCSparkMax(6, MotorType.kBrushless);
    OCSparkMax rightMotor = new OCSparkMax(5, MotorType.kBrushless);

    public Intake(){
        OCConfig.configMotors(kMotorFreeLimit, kMotorStallLimit, kRampRate, leftMotor,rightMotor);
		OCConfig.setStatusNormal(leftMotor, rightMotor);
		OCConfig.saveConfig(leftMotor, rightMotor);


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

    public boolean stallDetection(double volts){
        return false;
    }
}

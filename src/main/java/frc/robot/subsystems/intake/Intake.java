package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OCConfig;
import frc.robot.util.OCSparkMax;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase{
    
    OCSparkMax leftMotor = new OCSparkMax(11, MotorType.kBrushless);
    OCSparkMax rightMotor = new OCSparkMax(12, MotorType.kBrushless);

    public Intake(){
        OCConfig.configMotors(kMotorStallLimit, kMotorStallLimit, kRampRate, leftMotor, rightMotor);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor, true);
        OCConfig.setStatusNothing(rightMotor);
		OCConfig.saveConfig(leftMotor, rightMotor);

        OCConfig.setIdleMode(IdleMode.kBrake, leftMotor, rightMotor);
    }
 
    /**
     * Sets the voltage of the intake.
     * @param volts Voltage to set the intake to.
     */
    public void setVoltage(double volts){
        leftMotor.setVoltage(volts);
    }

    public boolean stallDetection(double volts){
        return false;
    }


    //---- Command Factories
    /**
     * Returns a RunCommand that sets the voltage of the intake.
     */
    public CommandBase setVoltageC(double volts) {
        return run(()->setVoltage(volts));
    }
    /**
     * Returns a RunCommand that sets the voltage for intaking gamepieces.
     */
    public CommandBase setVoltageInC() {
        return run(()->setVoltage(5));
    }
    /**
     * Returns a RunCommand that sets the voltage for outtaking gamepieces.
     */
    public CommandBase setVoltageOutC() {
        return run(()->setVoltage(-3));
    }
}

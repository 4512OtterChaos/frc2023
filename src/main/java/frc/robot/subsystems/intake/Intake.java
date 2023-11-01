package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OCConfig;
import frc.robot.util.OCSparkMax;
import frc.robot.util.OCSparkMax.OCRelativeEncoder;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase{
    
    OCSparkMax leftMotor = new OCSparkMax(6, MotorType.kBrushless);
    OCSparkMax rightMotor = new OCSparkMax(9, MotorType.kBrushless);

    OCRelativeEncoder leftEncoder = leftMotor.getEncoder();
    OCRelativeEncoder rightEncoder = rightMotor.getEncoder();
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.25, 0.005);
    private PIDController leftPid = new PIDController(0.005, 0, 0);
    private PIDController rightPid = new PIDController(0.005, 0, 0);
    boolean isManual = true;
    double lastIntakeTime = 0;

    public Intake(){
        leftMotor.setCANTimeout(100);
        rightMotor.setCANTimeout(100);
        OCConfig.configMotors(kMotorStallLimit, kMotorStallLimit, kRampRate, leftMotor, rightMotor);
        OCConfig.setStatusNormal(leftMotor, rightMotor);
        // leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
        // leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        // leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        // leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        // leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        // leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        // leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
        // rightMotor.follow(leftMotor, true);
        // OCConfig.setStatusNothing(rightMotor);
        OCConfig.setIdleMode(IdleMode.kBrake, leftMotor, rightMotor);
        leftMotor.setCANTimeout(0);
        rightMotor.setCANTimeout(0);
		OCConfig.saveConfig(leftMotor, rightMotor);
    }

    @Override
    public void periodic() {
        if(!isManual) {
            leftMotor.setVoltage(ff.calculate(leftPid.getSetpoint())+leftPid.calculate(leftEncoder.getVelocity()));
            rightMotor.setVoltage(ff.calculate(rightPid.getSetpoint())+rightPid.calculate(rightEncoder.getVelocity()));
        }
    }
 
    /**
     * Sets the voltage of the intake.
     * @param volts Voltage to set the intake to.
     */
    public void setVoltage(double volts){
        isManual = true;
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
        // leftMotor.setVoltage(0);
        // rightMotor.setVoltage(0);

    }

    public void setVelocity(double rpm) {
        // isManual = false;
        leftPid.setSetpoint(rpm);
        rightPid.setSetpoint(rpm);
        // leftPid.setSetpoint(0);
        // rightPid.setSetpoint(0);
    }

    public boolean stallDetection(){
        if ((getLeftCurrent() > kMotorStallDetection) && ((Timer.getFPGATimestamp()-0.5)>lastIntakeTime)){
            return true;
        }
        return false;
    }

    public double getLeftCurrent(){
        return leftMotor.getOutputCurrent();
    }
    
    public double getRightCurrent(){
        return rightMotor.getOutputCurrent();
    }

    public void setLastIntakeTime(){
        lastIntakeTime = Timer.getFPGATimestamp();
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
        return run(()->setVoltage(2.5));
    }
    /**
     * Returns a RunCommand that sets the voltage for outtaking gamepieces.
     */
    public CommandBase setVoltageOutC() {
        return run(()->setVoltage(-1.9));
    }

    public CommandBase setVelocityC(double rpm) {
        return run(()->setVelocity(rpm));
    }
    public CommandBase setVelocityInC() {
        return setVelocityC(300);
    }
    public CommandBase setVelocityOutC() {
        return setVelocityC(-300);
    }

}

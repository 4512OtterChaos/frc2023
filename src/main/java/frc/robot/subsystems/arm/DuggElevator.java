package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathHelp;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static frc.robot.subsystems.arm.ArmConstants.*;

import java.lang.Math;


public class DuggElevator extends SubsystemBase {
    VictorSP motorA = new VictorSP(0);
    VictorSP motorB = new VictorSP(1);
    
    private Encoder encoder = new Encoder(6,7);
    ProfiledPIDController pid = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.5));
    boolean isManual = true;
    
    DigitalInput topSwitch = new DigitalInput(1);
    DigitalInput bottomSwitch = new DigitalInput(8);

    Mechanism2d mechanism2d = new Mechanism2d(2,4, new Color8Bit(0, 100, 150));
    MechanismRoot2d mechRoot = mechanism2d.getRoot("elevator", 1.1, 0.5);

    MechanismLigament2d setpointCarriage = mechRoot.append(new MechanismLigament2d("setpointCarriage", 0, 90, 7, new Color8Bit(150,0,0)));
    MechanismLigament2d setpointIntake = setpointCarriage.append(new MechanismLigament2d("setpointIntake", Units.inchesToMeters(18), 90, 7, new Color8Bit(150,0,0)));


    MechanismLigament2d mechCarriage = mechRoot.append(new MechanismLigament2d("carriage", 0, 90));
    MechanismLigament2d mechIntake = mechCarriage.append(new MechanismLigament2d("intake", Units.inchesToMeters(18), 90));


    public DuggElevator(){
        encoder.setDistancePerPulse(kEncoderDistPerPulse);
        pid.setTolerance(Units.inchesToMeters(2), Units.inchesToMeters(5));

        SmartDashboard.putData("Mech2d", mechanism2d);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position Meters", getPosMeters());
        SmartDashboard.putNumber("Elevator Position Inches", Units.metersToInches(getPosMeters()));
        SmartDashboard.putNumber("Elevator Target Position", pid.getSetpoint().position);
        
        setpointCarriage.setLength(pid.getSetpoint().position);
        mechCarriage.setLength(getPosMeters()); 


        // if(!isManual){
        //     double pidVolts = pid.calculate(getPosMeters());
        //     var setpoint = pid.getSetpoint();
        //     double ffVolts = kFF.calculate(setpoint.velocity);
        //     setVoltage(pidVolts+ffVolts);
        // }

    }

    public void setSpeed(double percent) {
        isManual = true;
        setVoltage(percent*12);
    }
    public CommandBase setSpeedC(double percent) {
        return runEnd(()->setSpeed(percent), ()->setSpeed(0));
    }
    private void setVoltage(double volts) {
        if(topSwitch.get()){
            volts = Math.min(volts, 0);

        }
        if(bottomSwitch.get()){
            volts = Math.max(volts, 0);
        }
        volts = MathUtil.clamp(volts, -9, 9);
        motorA.setVoltage(volts);
        motorB.setVoltage(volts);    
    }
    public void setPosMeters(double posMeters) {
        isManual = false;
        pid.setGoal(posMeters);
    }
    public CommandBase setPosMetersC(double posMeters) {
        return run(()->setPosMeters(posMeters)).until(()->pid.atGoal()) ;
    }
    


    public double getPosMeters(){
        return encoder.getDistance();
    }
    

    PWMSim motorSim = new PWMSim(motorA);
    EncoderSim encoderSim = new EncoderSim(encoder);
    DIOSim topSwitchSim = new DIOSim(topSwitch);
    DIOSim bottomSwitchSim = new DIOSim(bottomSwitch);
    ElevatorSim elevatorSim = new ElevatorSim(
        LinearSystemId.identifyPositionSystem(
            kShoulderFF.kv, 
            kShoulderFF.ka
        ),
        DCMotor.getCIM(2), 
        10, 
        Units.inchesToMeters(1), 
        0, 
        kTravelHeight, 
        true
    );

    @Override
    public void simulationPeriodic() {
        // get "voltage" after static friction
        double voltage = motorSim.getSpeed()*12;
        System.err.println("elev pwm speed: "+motorSim.getSpeed());
        // if(voltage >= 0) voltage = Math.max(0, voltage-kFF.ks);
        // else voltage = Math.min(0, voltage+kFF.ks);

        // apply this voltage to the simulated physics model
        elevatorSim.setInput(voltage);
        elevatorSim.update(0.02);
        // update our sensors with the results
        encoderSim.setDistance(elevatorSim.getPositionMeters());
        encoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());
        topSwitchSim.setValue(encoder.getDistance()>=kTravelHeight);
        bottomSwitchSim.setValue(encoder.getDistance()<=0);
    }
}

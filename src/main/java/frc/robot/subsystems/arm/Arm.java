// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.arm.ArmConstants.*;


public class Arm extends SubsystemBase {
	VictorSP motorA = new VictorSP(0);
	VictorSP motorB = new VictorSP(1);
	
	private Encoder encoder = new Encoder(6,7);
	ProfiledPIDController pid = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.5));
	public boolean isActive = false;
	

	Mechanism2d mech = new Mechanism2d(2.5, 2.5, new Color8Bit(0, 100, 150));
	MechanismRoot2d mechRoot = mech.getRoot("arm", 2, 0.5);
	
	MechanismLigament2d setpointArmBase = mechRoot.append(new MechanismLigament2d("setpointArmBase", Units.inchesToMeters(40), 90, 6, new Color8Bit(150,0,0)));
	MechanismLigament2d setpointArm = setpointArmBase.append(new MechanismLigament2d("setpointArm", Units.inchesToMeters(30), 90, 6, new Color8Bit(150,0,0)));
	MechanismLigament2d setpointExtension = setpointArm.append(new MechanismLigament2d("setpointExtension", Units.inchesToMeters(18), 0, 4, new Color8Bit(180,0,0)));
	MechanismLigament2d setpointWrist = setpointExtension.append(new MechanismLigament2d("setpointWrist", Units.inchesToMeters(18), 0, 5, new Color8Bit(180,0,0)));
	
	MechanismLigament2d mechArmBase = mechRoot.append(new MechanismLigament2d("base", Units.inchesToMeters(40), 90, 10, new Color8Bit(0,0,150)));
	MechanismLigament2d mechArm = mechArmBase.append(new MechanismLigament2d("arm", Units.inchesToMeters(30), 90, 10, new Color8Bit(0,0,150)));
	MechanismLigament2d mechExtension = mechArm.append(new MechanismLigament2d("armExtension", Units.inchesToMeters(18), 0, 7, new Color8Bit(0,0,180)));
	MechanismLigament2d mechWrist = mechExtension.append(new MechanismLigament2d("armWrist", Units.inchesToMeters(16), 0, 8, new Color8Bit(0,0,180)));

	PWMSim motorSim = new PWMSim(motorA);
	EncoderSim encoderSim = new EncoderSim(encoder);
	SingleJointedArmSim armSim = new SingleJointedArmSim(
		LinearSystemId.identifyPositionSystem(
            kFF.kv, 
            kFF.ka
        ),
		DCMotor.getCIM(2), 
		100, 
		1,
		Units.degreesToRadians(-45),
		Units.degreesToRadians(30), 
		Units.lbsToKilograms(12), 
		true);
	
	
	public Arm() {
		SmartDashboard .putData("Mech2d", mech);
		encoder.setDistancePerPulse(2*(Math.PI)/kEncoderPPR);
	}
	
	@Override
	public void periodic() {
		mechArm.setAngle(90-Units.radiansToDegrees(encoder.getDistance()));
		setpointArm.setAngle(90-Units.radiansToDegrees(pid.getGoal().position));
		SmartDashboard.putNumber("Arm Angle: Radians", getPosRadians());
		SmartDashboard.putNumber("Arm Angle: Degrees", Units.radiansToDegrees(getPosRadians()));


		double pidVolts = pid.calculate(getPosRadians());
        var setpoint = pid.getSetpoint();
        double ffVolts = kFF.calculate(setpoint.position, setpoint.velocity);
        setVoltage(pidVolts+ffVolts);
	}

	public void setSpeed(double percent){
        setVoltage(percent*12);
	}

	public CommandBase setSpeedC(double percent) {
        return runEnd(()->setSpeed(percent), ()->setSpeed(0));
    }

	public void setVoltage(double volts){
		volts = MathUtil.clamp(volts, -9, 9);
        motorA.setVoltage(volts);
        motorB.setVoltage(volts);
	}

	public double getPosRadians(){
        return encoder.getDistance();
    }
    public void setPosRadians(double posRadians) {
        pid.setGoal(posRadians);
    }
    public CommandBase setPosRadiansC(double posRadians) {
        return run(()->setPosRadians(posRadians)).until(()->pid.atGoal());
    }
    

	public void simulationPeriodic() {
		// get "voltage" after static friction
		double voltage = motorSim.getSpeed()*12;
		// if(voltage >= 0) voltage = Math.max(0, voltage-kFF.ks);
		// else voltage = Math.min(0, voltage+kFF.ks);
		
		// apply this voltage to the simulated physics model
		armSim.setInput(voltage);
		armSim.update(0.02);
		// update our sensors with the results
		encoderSim.setDistance(armSim.getAngleRads());
		encoderSim.setRate(armSim.getVelocityRadPerSec());
		
	}
}

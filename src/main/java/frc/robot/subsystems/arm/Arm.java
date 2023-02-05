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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.arm.ArmConstants.*;

public class Arm extends SubsystemBase {
	VictorSP motorA = new VictorSP(0);
	VictorSP motorB = new VictorSP(1);
	VictorSP motorC = new VictorSP(2);
	VictorSP motorD = new VictorSP(3);
	VictorSP motorE = new VictorSP(4);

	private Encoder shoulderEncoder = new Encoder(6, 7);
	private Encoder wristEncoder = new Encoder(8, 9);

	ProfiledPIDController shoulderPid = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.5));
	ProfiledPIDController wristPid = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.5));

	// Ligaments of arm simulation, rooted to different each and every ligament.
	Mechanism2d mech = new Mechanism2d(2.5, 2.5, new Color8Bit(0, 100, 150));
	MechanismRoot2d mechRoot = mech.getRoot("arm", 1.25, 0.1);

	MechanismLigament2d setpointArmBase = mechRoot.append(
			new MechanismLigament2d("setpointArmBase", Units.inchesToMeters(40), 90, 6, new Color8Bit(150, 0, 0)));
	MechanismLigament2d setpointArm = setpointArmBase
			.append(new MechanismLigament2d("setpointArm", Units.inchesToMeters(30), 90, 6, new Color8Bit(150, 0, 0)));
	MechanismLigament2d setpointExtension = setpointArm.append(
			new MechanismLigament2d("setpointExtension", Units.inchesToMeters(18), 0, 4, new Color8Bit(180, 0, 0)));
	MechanismLigament2d setpointWrist = setpointExtension
			.append(new MechanismLigament2d("setpointWrist", Units.inchesToMeters(18), 0, 5, new Color8Bit(180, 0, 0)));

	MechanismLigament2d mechArmBase = mechRoot
			.append(new MechanismLigament2d("base", Units.inchesToMeters(40), 90, 10, new Color8Bit(0, 0, 150)));
	MechanismLigament2d mechArm = mechArmBase
			.append(new MechanismLigament2d("arm", Units.inchesToMeters(30), 90, 10, new Color8Bit(0, 0, 150)));
	MechanismLigament2d mechExtension = mechArm
			.append(new MechanismLigament2d("armExtension", Units.inchesToMeters(18), 0, 7, new Color8Bit(0, 0, 180)));
	MechanismLigament2d mechWrist = mechExtension
			.append(new MechanismLigament2d("armWrist", Units.inchesToMeters(16), 0, 8, new Color8Bit(0, 0, 180)));

	PWMSim shoulderMotorSim = new PWMSim(motorA);
	PWMSim wristMotorSim = new PWMSim(motorD);
	EncoderSim shoulderEncoderSim = new EncoderSim(shoulderEncoder);
	EncoderSim wristEncoderSim = new EncoderSim(wristEncoder);

	DoubleSolenoid exstensionPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

	SingleJointedArmSim shoulderSim = new SingleJointedArmSim(
			LinearSystemId.identifyPositionSystem(
					kShoulderFF.kv,
					kShoulderFF.ka),
			DCMotor.getCIM(2),
			100,
			1,
			Units.degreesToRadians(-90),
			Units.degreesToRadians(30),
			Units.lbsToKilograms(12),
			true);

	SingleJointedArmSim wristSim = new SingleJointedArmSim(
			LinearSystemId.identifyPositionSystem(
					kWristFF.kv,
					kWristFF.ka),
			DCMotor.getCIM(2),
			100,
			1,
			Units.degreesToRadians(-90),
			Units.degreesToRadians(90),
			Units.lbsToKilograms(7),
			true);

	public Arm() {
		SmartDashboard.putData("Mech2d", mech);
		shoulderEncoder.setDistancePerPulse(2 * (Math.PI) / kEncoderPPR);
		wristEncoder.setDistancePerPulse(2 * (Math.PI) / kEncoderPPR);
		wristEncoder.setReverseDirection(true);
		motorD.setInverted(false);
		motorE.setInverted(false);
		exstensionPiston.set(Value.kReverse);
	}

	@Override
	public void periodic() {
		mechArm.setAngle(90 - Units.radiansToDegrees(shoulderEncoder.getDistance()));
		mechWrist.setAngle(-Units.radiansToDegrees(wristEncoder.getDistance()));
		if (exstensionPiston.get()==Value.kForward){
			mechExtension.setLength(Units.inchesToMeters(18));
			setpointExtension.setLength(Units.inchesToMeters(18));
		}
		if (exstensionPiston.get()==Value.kReverse){
			mechExtension.setLength(Units.inchesToMeters(0));
			setpointExtension.setLength(Units.inchesToMeters(0));
		}
		
		setpointArm.setAngle(90 - Units.radiansToDegrees(shoulderPid.getGoal().position));
		setpointWrist.setAngle(-Units.radiansToDegrees(wristPid.getGoal().position));

		SmartDashboard.putNumber("Shoulder Angle: Radians", getShoulderPosRadians());
		SmartDashboard.putNumber("Shoulder Angle: Degrees", Units.radiansToDegrees(getShoulderPosRadians()));
		SmartDashboard.putNumber("Wrist Angle: Radians", getWristPosRadians());
		SmartDashboard.putNumber("Wrist Angle: Degrees", Units.radiansToDegrees(getWristPosRadians()));
		SmartDashboard.putNumber("Wrist Setpoint: Degrees", Units.radiansToDegrees(getWristPosRadians()));

		

		double shoulderPidVolts = shoulderPid.calculate(getShoulderPosRadians());
		var shoulderSetpoint = shoulderPid.getSetpoint();
		double shoulderFFVolts = kShoulderFF.calculate(shoulderSetpoint.position, shoulderSetpoint.velocity);
		double shoulderVolts = shoulderPidVolts + shoulderFFVolts;

		double wristPidVolts = wristPid.calculate(getWristPosRadians());
		var wristSetpoint = wristPid.getSetpoint();
		double wristFFVolts = kWristFF.calculate(wristSetpoint.position, wristSetpoint.velocity);
		double wristVolts = wristPidVolts + wristFFVolts;

		double testAngleDeg = -30;
		if (Units.radiansToDegrees(getShoulderPosRadians()) <= testAngleDeg){
			shoulderVolts = MathUtil.clamp(shoulderVolts, kShoulderFF.kg*Math.cos(Math.toRadians(testAngleDeg)), 12);
			wristVolts = MathUtil.clamp(wristVolts, kWristFF.kg*Math.cos(Math.toRadians(testAngleDeg)), 12);
		}

		setWristVoltage(wristVolts);
		setShoulderVoltage(shoulderVolts);
	}


	
	// private boolean safety(){
	// 	double shoulderAngle = 90 - Math.abs(Units.radiansToDegrees(getWristPosRadians()));
	// 	double wristAngle = shoulderAngle + Math.abs(Units.radiansToDegrees(getWristPosRadians()));

	// 	double armBaseHeight = 40;
	// 	double H = armBaseHeight*Math.cos(shoulderAngle);

	// 	double h = 48;
	// 	double M = (h/H)*armBaseHeight;
		


	// 	double K = 16;
	// 	double m = K/Math.cos(wristAngle);

	// 	if (M+m>=35){
	// 		return true;
	// 	}
	// 	else{
	// 		return false;
	// 	}
		
	// }

	public void setSpeed(double percent) {
		setShoulderVoltage(percent * 12);
	}
	// public void armSafety(){
	// 	double 
	// }
	public CommandBase setSpeedC(double percent) {
		return runEnd(() -> setSpeed(percent), () -> setSpeed(0));
	}

	public void setShoulderVoltage(double volts) {
		volts = MathUtil.clamp(volts, -9, 9);
		motorA.setVoltage(volts);
		motorB.setVoltage(volts);
	}

	public void setWristVoltage(double volts) {
		volts = MathUtil.clamp(volts, -9, 9);
		motorD.setVoltage(volts);
		motorE.setVoltage(volts);
	}

	public double getShoulderPosRadians() {
		return shoulderEncoder.getDistance();
	}

	public double getWristPosRadians() {
		return wristEncoder.getDistance();
	}

	public void setShoulderPosRadians(double posRadians) {
		shoulderPid.setGoal(posRadians);
	}

	public CommandBase setShoulderPosRadiansC(double posRadians) {
		return run(() -> setShoulderPosRadians(posRadians)).until(() -> shoulderPid.atGoal());
	}

	public void setWristPosRadians(double posRadians) {
		wristPid.setGoal(posRadians);
	}

	public CommandBase setWristPosRadiansC(double posRadians) {
		return run(() -> setWristPosRadians(posRadians)).until(() -> wristPid.atGoal());
	}

	public void toggleExstensionExtended(){
		exstensionPiston.toggle();
	}

	public CommandBase toggleExstensionExtendedC(){
		return runOnce(()-> toggleExstensionExtended());
	}
	
	public void exstensionExtended(Value exstensionState){
		exstensionPiston.set(exstensionState);
	}

	public CommandBase exstensionExtendedC(Value exstensionState){
		return run(()-> exstensionExtended(exstensionState));
	}
	
	public Value getExtensionState(){
		return exstensionPiston.get();
	}

	public void setArmState(double shoulderPosRadians, double wristPosRadians, Value extensionState){
		setShoulderPosRadians(shoulderPosRadians);
		setWristPosRadians(wristPosRadians);
		exstensionExtended(extensionState);
	}


	public CommandBase setArmStateC(double shoulderPosRadians, double wristPosRadians, Value extensionState){
		return run(()->setArmState(shoulderPosRadians, wristPosRadians, extensionState));
	}

	

	public void simulationPeriodic() {
		// get "voltage" after static friction
		double shoulderVoltage = shoulderMotorSim.getSpeed() * 12;
		double wristVoltage = wristMotorSim.getSpeed() * 12;

		// if(voltage >= 0) voltage = Math.max(0, voltage-kFF.ks);
		// else voltage = Math.min(0, voltage+kFF.ks);

		// apply this voltage to the simulated physics model
		shoulderSim.setInput(shoulderVoltage);
		shoulderSim.update(0.02);

		wristSim.setInput(wristVoltage);
		wristSim.update(0.02);
		// update our sensors with the results
		shoulderEncoderSim.setDistance(shoulderSim.getAngleRads());
		shoulderEncoderSim.setRate(shoulderSim.getVelocityRadPerSec());

		wristEncoderSim.setDistance(wristSim.getAngleRads());
		wristEncoderSim.setRate(wristSim.getVelocityRadPerSec());

	}
}

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
		SmartDashboard.putData("Arm Subsystem", this);
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

		SmartDashboard.putNumber("Shoulder Angle: Degrees", Units.radiansToDegrees(getShoulderPosRadians()));
		SmartDashboard.putNumber("Shoulder Goal: Degrees", Units.radiansToDegrees(shoulderPid.getGoal().position));
		SmartDashboard.putNumber("Wrist Angle: Degrees", Units.radiansToDegrees(getWristPosRadians()));
		SmartDashboard.putNumber("Wrist Goal: Degrees", Units.radiansToDegrees(wristPid.getGoal().position));

		

		double shoulderPidVolts = shoulderPid.calculate(getShoulderPosRadians());
		var shoulderSetpoint = shoulderPid.getSetpoint();
		double shoulderFFVolts = kShoulderFF.calculate(shoulderSetpoint.position, shoulderSetpoint.velocity);
		double shoulderVolts = shoulderPidVolts + shoulderFFVolts;

		double wristPidVolts = wristPid.calculate(getWristPosRadians());
		var wristSetpoint = wristPid.getSetpoint();
		double wristFFVolts = kWristFF.calculate(wristSetpoint.position, wristSetpoint.velocity);
		double wristVolts = wristPidVolts + wristFFVolts;

		double testAngleDeg = -40;
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

	// public void setSpeed(double percent) {
	// 	setShoulderVoltage(percent * 12);
	// }
	// public void armSafety(){
	// 	double 
	// }
	// public CommandBase setSpeedC(double percent) {
	// 	return runEnd(() -> setSpeed(percent), () -> setSpeed(0));
	// }

	/**
	 * Sets the voltage for shoulder motors (angle at the top of the arm).
	 * @param volts The voltage to set the shoulder motors to (between -12 and 12).
	 */
	public void setShoulderVoltage(double volts) {
		motorA.setVoltage(volts);
		motorB.setVoltage(volts);
	}

	/**
	 * Sets the voltage for the wrist motors (angle at the bottom of the arm connected to the intake).
	 * @param volts The voltage to set the wrist motors to (between -12 and 12).
	 */
	public void setWristVoltage(double volts) {
		motorD.setVoltage(volts);
		motorE.setVoltage(volts);
	}

	/**
	 * 
	 * @return The shoulder position in radians.
	 */
	public double getShoulderPosRadians() {
		return shoulderEncoder.getDistance();
	}


	/**
	 * 
	 * @return The wrist position in radians.
	 */
	public double getWristPosRadians() {
		return wristEncoder.getDistance();
	}

	/**
	 * Sets the position of the shoulder while clamping the angle to stay within a safe range.
	 * @param posRadians The radian value to set the shoulder angle to (between -40 and 30).
	 */
	public void setShoulderPosRadians(double posRadians) {
		double clampedPosRadians = MathUtil.clamp(posRadians, Units.degreesToRadians(-40), Units.degreesToRadians(30));
		setWristPosRadians(Units.radiansToDegrees(wristPid.getGoal().position));
		shoulderPid.setGoal(clampedPosRadians);
	}

	/**
	 * Sets the position of the shoulder in a command.
	 * @param posRadians The radian value to set the shoulder angle to (between -40 and 30).
	 * @return The command to set the position of the shoulder.
	 */
	public CommandBase setShoulderPosRadiansC(double posRadians) {
		return run(() -> setShoulderPosRadians(posRadians)).until(() -> shoulderPid.atGoal());
	}

	/**
	 * Sets the position of the wrist while clamping the angle to stay within a safe range that changes depending on the shoulder angle.
	 * @param posRadians The radian value to set the wrist angle to (between -45 and 45).
	 */
	public void setWristPosRadians(double posRadians) {
		double lowerClamp=-45;
		if (Units.radiansToDegrees(shoulderPid.getGoal().position)<-30){
			lowerClamp=-Units.radiansToDegrees(shoulderPid.getGoal().position);
		}
		double clampedPosRadians = MathUtil.clamp(posRadians, Units.degreesToRadians(lowerClamp), Units.degreesToRadians(45));
		wristPid.setGoal(clampedPosRadians);
	}

	/**
	 * Sets the position of the wrist in a command.
	 * @param posRadians The radian value to set the wrist angle to (between -45 and 45).
	 * @return The command to set the position of the wrist.
	 */
	public CommandBase setWristPosRadiansC(double posRadians) {
		return run(() -> setWristPosRadians(posRadians)).until(() -> wristPid.atGoal());
	}

	/**
	 * Toggles the extension piston between forward (out) and reverse (in).
	 */
	public void toggleExstensionExtended(){
		exstensionPiston.toggle();
	}

	/**
	 * Toggles the extension piston between forward (out) and reverse (in) in a command.
	 * @return The command to toggle the extension piston.
	 */
	public CommandBase toggleExstensionExtendedC(){
		return runOnce(()-> toggleExstensionExtended());
	}
	
	/**
	 * Sets the state of the exstension pistion to the desired state.
	 * @param exstensionState The value to set the exstension pistion to (forward, off or reverse).
	 */
	public void exstensionExtended(Value exstensionState){
		exstensionPiston.set(exstensionState);
	}

	/**
	 * Sets the state of the exstension pistion to the desired state in a command.
	 * @param exstensionState The value to set the exstension pistion to (forward, off or reverse).
	 * @return The command to set the exstension pistion.
	 */
	public CommandBase exstensionExtendedC(Value exstensionState){
		return run(()-> exstensionExtended(exstensionState));
	}
	
	/**
	 * 
	 * @return The current state of the exstension piston.
	 */
	public Value getExtensionState(){
		return exstensionPiston.get();
	}

	/**
	 * Sets the state of the entire arm.
	 * @param shoulderPosRadians The radian value to set the shoulder angle to (between -40 and 30).
	 * @param wristPosRadians The radian value to set the wrist angle to (between -45 and 45).
	 * @param extensionState The value to set the exstension pistion to (forward, off or reverse).
	 */
	public void setArmState(double shoulderPosRadians, double wristPosRadians, Value extensionState){
		setShoulderPosRadians(shoulderPosRadians);
		setWristPosRadians(wristPosRadians);
		exstensionExtended(extensionState);
	}

	/**
	 * Sets the state of the entire arm in a command.
	 * @param shoulderPosRadians The radian value to set the shoulder angle to (between -40 and 30).
	 * @param wristPosRadians The radian value to set the wrist angle to (between -45 and 45).
	 * @param extensionState The value to set the exstension pistion to (forward, off or reverse).
	 * @return The command to set the state of the arm.
	 */
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

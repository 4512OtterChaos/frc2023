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
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.*;

import static frc.robot.subsystems.arm.ArmConstants.*;

@Log.Exclude
public class Arm extends SubsystemBase implements Loggable{
	VictorSP motorA = new VictorSP(0);
	VictorSP motorB = new VictorSP(1);
	VictorSP motorC = new VictorSP(2);
	VictorSP motorD = new VictorSP(3);
	VictorSP motorE = new VictorSP(4);

	private Encoder shoulderEncoder = new Encoder(6, 7);
	private Encoder wristEncoder = new Encoder(8, 9);
	
	public ProfiledPIDController shoulderPid = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(0.7, 0.7));
	public ProfiledPIDController wristPid = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(0.8, 0.8));

	// Ligaments of arm simulation, rooted to different each and every ligament.
	Mechanism2d mech = new Mechanism2d(2.5, 2.5, new Color8Bit(0, 100, 150));
	MechanismRoot2d mechRoot = mech.getRoot("arm", 1.25, 0.1);

	MechanismLigament2d setpointArmBase = mechRoot.append(
			new MechanismLigament2d("setpointArmBase", Units.inchesToMeters(40), 90, 6, new Color8Bit(150, 0, 0)));
	MechanismLigament2d setpointArm = setpointArmBase
			.append(new MechanismLigament2d("setpointArm", Units.inchesToMeters(30), -90, 6, new Color8Bit(150, 0, 0)));
	MechanismLigament2d setpointExtension = setpointArm.append(
			new MechanismLigament2d("setpointExtension", Units.inchesToMeters(18), 0, 4, new Color8Bit(180, 0, 0)));
	MechanismLigament2d setpointWrist = setpointExtension
			.append(new MechanismLigament2d("setpointWrist", Units.inchesToMeters(18), 0, 5, new Color8Bit(180, 0, 0)));

	MechanismLigament2d mechArmBase = mechRoot
			.append(new MechanismLigament2d("base", Units.inchesToMeters(40), 90, 10, new Color8Bit(0, 0, 150)));
	MechanismLigament2d mechArm = mechArmBase
			.append(new MechanismLigament2d("arm", Units.inchesToMeters(30), -90, 10, new Color8Bit(0, 0, 150)));
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
			true
	);

	SingleJointedArmSim wristSim = new SingleJointedArmSim(
			LinearSystemId.identifyPositionSystem(
					kWristFF.kv,
					kWristFF.ka),
			DCMotor.getCIM(2),
			100,
			1,
			Units.degreesToRadians(-45),
			Units.degreesToRadians(90),
			true
	);
	
	public double wristMinimumAngle = kWristMinimumAngle;
    public double wristMaximumAngle = kWristMaximumAngle;
	
	public double shoulderWristLevel = kShoulderWristLevel;
	public double shoulderExstensionWristLevel = kShoulderWristLevel;

    public double shoulderMinimumAngle = kShoulderMinimumAngle;
    public double shoulderMinimumAngleWrist = kShoulderMinimumAngleWrist;
    public double shoulderMinimumAngleExtension = kShoulderMinimumAngleExtension;
    public double shoulderMinimumAngleExtensionWrist = kShoulderMinimumAngleExtensionWrist;
    public double shoulderMaximumAngle = kShoulderMaximumAngle;
	public static final double kWristMinimumAngle = 0;
	@Config(defaultValueNumeric = kWristMinimumAngle)
	void configWristMinimumAngle(double wristMinimumAngle) {
		this.wristMinimumAngle = (wristMinimumAngle);
	}
	@Config(defaultValueNumeric = kWristMaximumAngle)
	void configWristMaximumAngle(double wristMaximumAngle) {
		this.wristMaximumAngle = wristMaximumAngle;
	}
	@Config(defaultValueNumeric = kShoulderWristLevel)
	void configShoulderWristLevel(double shoulderWristLevel) {
		this.shoulderWristLevel = shoulderWristLevel;
	}
	@Config(defaultValueNumeric = kShoulderExstensionWristLevel)
	void configShoulderExstensionWristLevel(double shoulderExstensionWristLevel) {
		this.shoulderExstensionWristLevel = shoulderExstensionWristLevel;
	}
	@Config(defaultValueNumeric = kShoulderMinimumAngleWrist)
	void configShoulderMinimumAngleWrist(double shoulderMinimumAngleWrist) {
		this.shoulderMinimumAngleWrist = shoulderMinimumAngleWrist;
	}
	@Config(defaultValueNumeric = kShoulderMinimumAngle)
	void configShoulderMinimumAngle(double shoulderMinimumAngle) {
		this.shoulderMinimumAngle = shoulderMinimumAngle;
	}
	@Config(defaultValueNumeric = kShoulderMinimumAngleExtension)
	void configShoulderMinimumAngleExtension(double shoulderMinimumAngleExtension) {
		this.shoulderMinimumAngleExtension = shoulderMinimumAngleExtension;
	}
	@Config(defaultValueNumeric = kShoulderMinimumAngleExtensionWrist)
	void configShoulderMinimumAngleExtensionWrist(double shoulderMinimumAngleExtensionWrist) {
		this.shoulderMinimumAngleExtensionWrist = shoulderMinimumAngleExtensionWrist;
	}
	@Config(defaultValueNumeric = kShoulderMaximumAngle)
	void configShoulderMaximumAngle(double shoulderMaximumAngle) {
		this.shoulderMaximumAngle = shoulderMaximumAngle;
	}

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
		mechArm.setAngle(-90 + Units.radiansToDegrees(shoulderEncoder.getDistance()));
		mechWrist.setAngle(Units.radiansToDegrees(wristEncoder.getDistance()));
		if (exstensionPiston.get()==Value.kForward){
			mechExtension.setLength(Units.inchesToMeters(18));
			setpointExtension.setLength(Units.inchesToMeters(18));
		}
		if (exstensionPiston.get()==Value.kReverse){
			mechExtension.setLength(Units.inchesToMeters(0));
			setpointExtension.setLength(Units.inchesToMeters(0));
		}
		
		setpointArm.setAngle(-90 + Units.radiansToDegrees(shoulderPid.getGoal().position));
		setpointWrist.setAngle(Units.radiansToDegrees(wristPid.getGoal().position));

		SmartDashboard.putNumber("Shoulder Angle: Degrees", Units.radiansToDegrees(getShoulderPosRadians()));
		SmartDashboard.putNumber("Shoulder Goal: Degrees", Units.radiansToDegrees(shoulderPid.getGoal().position));
		SmartDashboard.putNumber("Wrist Angle: Degrees", Units.radiansToDegrees(getWristPosRadians()));
		SmartDashboard.putNumber("Wrist Goal: Degrees", Units.radiansToDegrees(wristPid.getGoal().position));
		SmartDashboard.putBoolean("Extension State", getExtensionState());

		

		double shoulderPosRadians = getShoulderPosRadians();
		double wristPosRadians = getWristPosRadians();

		// Calculate shoulder volts using feedforward and PID
		var shoulderSetpoint = shoulderPid.getSetpoint();
		double shoulderVolts = shoulderPid.calculate(shoulderPosRadians) + kShoulderFF.calculate(shoulderSetpoint.position, shoulderSetpoint.velocity);
		
		double standardShoulderVolts = Math.cos(shoulderPosRadians)*kWristFF.kg;

		// Clamp shoulder position to an appropriate angle.
		double clampedShoulderPos = shoulderSafety(shoulderPosRadians, wristPosRadians, getExtensionState());

		// Clamp shoulder volts to have the shoulder stay inside/go into appropriate angle range.
		if (clampedShoulderPos > shoulderPosRadians){
			shoulderVolts = Math.max(standardShoulderVolts, shoulderVolts);
		}
		if (clampedShoulderPos<shoulderPosRadians){
			shoulderVolts = Math.min(standardShoulderVolts, shoulderVolts);
		}
		//Set shoulder motors to the clamped voltage.
		setShoulderVoltage(shoulderVolts);
		


		// Calculate wrist volts using feedforward and PID
		var wristSetpoint = wristPid.getSetpoint();
		double wristVolts = wristPid.calculate(getWristPosRadians()) + kWristFF.calculate(wristSetpoint.position, wristSetpoint.velocity);

		double standardWristVolts = Math.cos(wristPosRadians)*kWristFF.kg;

		// Clamp wrist position to an appropriate angle.
		double clampedWristPos = wristSafety(wristPosRadians, shoulderPosRadians, getExtensionState());
		// System.out.println("clampedWristPos: " + Units.radiansToDegrees(clampedWristPos));
		// Clamp wrist volts to have the wrist stay inside/go into appropriate angle range.
		if (clampedWristPos > wristPosRadians){
			wristVolts = Math.max(standardWristVolts, wristVolts);
		}
		if (clampedWristPos < wristPosRadians){
			wristVolts = Math.min(standardWristVolts, wristVolts);
		}
		//Set wrist motors to the clamped voltage.
		setWristVoltage(wristVolts);
		

	}


	
	private double shoulderSafety(double shoulderPosRadians, double wristPosRadians, boolean extensionState){
		double minimumPosRadians = shoulderMinimumAngle;
		// exstension out
		if (extensionState == true){
			// if wrist angle is less than level
			if (wristPosRadians<-shoulderPosRadians-Units.degreesToRadians(8)){
				minimumPosRadians = shoulderMinimumAngleExtensionWrist;
			}
			else{
				minimumPosRadians = shoulderMinimumAngleExtension;
			}
		}
		// if exstension in and wrist is less than level
		if (extensionState == false && wristPosRadians<-getShoulderPosRadians()-Units.degreesToRadians(5)){
			minimumPosRadians = shoulderMinimumAngleWrist;
			
		}
		return MathUtil.clamp(shoulderPosRadians, minimumPosRadians, shoulderMaximumAngle);
	}

	private double wristSafety(double wristPosRadians, double shoulderPosRadians, boolean extensionState){
		double minimumPosRadians = wristMinimumAngle;
		//exstension out and shoulder goal is below set angle
		if (extensionState == true && shoulderPid.getGoal().position < shoulderExstensionWristLevel){
			minimumPosRadians = -shoulderPosRadians;
		}
		//exstension in and shoulder goal is below set angle
		if (extensionState == false && shoulderPid.getGoal().position < shoulderWristLevel){
			minimumPosRadians = -shoulderPosRadians;
		}
		
		return MathUtil.clamp(wristPosRadians, minimumPosRadians, wristMaximumAngle);
	}

	/**
	 * Sets the voltage for shoulder motors (angle at the top of the arm).
	 * @param volts The voltage to set the shoulder motors to (between -12 and 12).
	 */
	public void setShoulderVoltage(double volts) {
		motorA.setVoltage(volts);
		motorB.setVoltage(volts);
	}

	/**
	 * Sets the position of the shoulder while clamping the angle to stay within a safe range.
	 * @param posRadians The radian value to set the shoulder angle to (between -90 and 30).
	 */
	public void setShoulderPosRadians(double posRadians) {
		// double clampedPosRadians = shoulderSafety(posRadians, getWristPosRadians() , getExtensionState());
		shoulderPid.setGoal(posRadians);
		setWristPosRadians(wristPid.getGoal().position);
	}

	/**
	 * Sets the position of the shoulder in a command.
	 * @param posRadians The radian value to set the shoulder angle to (between -90 and 30).
	 * @return The command to set the position of the shoulder.
	 */
	public CommandBase setShoulderPosRadiansC(double posRadians) {
		return run(() -> setShoulderPosRadians(posRadians)).until(() -> shoulderPid.atGoal());
	}

	/**
	 * 
	 * @return The shoulder position in radians.
	 */
	public double getShoulderPosRadians() {
		return shoulderEncoder.getDistance();
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
	 * Sets the position of the wrist while clamping the angle to stay within a safe range that changes depending on the shoulder angle.
	 * @param posRadians The radian value to set the wrist angle to (between -45 and 45).
	 */
	public void setWristPosRadians(double posRadians) {
		// double clampedPosRadians = wristSafety(posRadians, shoulderPid.getGoal().position, getExtensionState());
		wristPid.setGoal(posRadians);
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
	 * 
	 * @return The wrist position in radians.
	 */
	public double getWristPosRadians() {
		return wristEncoder.getDistance();
	}

	/**
	 * Toggles the extension piston between forward (out) and reverse (in).
	 */
	public void toggleExstensionExtended(){
		if (getExtensionState() == true || getShoulderPosRadians() > Units.degreesToRadians(-40)){
			 exstensionPiston.toggle();
		}
		setShoulderPosRadians(shoulderPid.getGoal().position);
		setWristPosRadians(wristPid.getGoal().position);
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
	public void setExstensionExtended(boolean exstended){
		if (exstended==true && getShoulderPosRadians() > Units.degreesToRadians(-40)){
			exstensionPiston.set(Value.kForward);
		}
		else{
			exstensionPiston.set(Value.kReverse);
		}
		setShoulderPosRadians(shoulderPid.getGoal().position);
		setWristPosRadians(wristPid.getGoal().position);
	}

	/**
	 * Sets the state of the exstension pistion to the desired state in a command.
	 * @param exstended The value to set the exstension pistion to (forward, off or reverse).
	 * @return The command to set the exstension pistion.
	 */
	public CommandBase setExstensionExtendedC(boolean exstended){
		return run(()-> setExstensionExtended(exstended));
	}
	
	/**
	 * 
	 * @return Whether the extension is extended.
	 */
	public boolean getExtensionState(){
		if (exstensionPiston.get()==Value.kForward){
			return true;
		}
		else{
			return false;
		}
	}

	/**
	 * Sets the state of the entire arm.
	 * @param shoulderPosRadians The radian value to set the shoulder angle to (between -90 and 30).
	 * @param wristPosRadians The radian value to set the wrist angle to (between -45 and 45).
	 * @param extended The value to set the exstension pistion to (forward, off or reverse).
	 */
	public void setArmState(double shoulderPosRadians, double wristPosRadians, boolean extended){
		setShoulderPosRadians(shoulderPosRadians);
		setWristPosRadians(wristPosRadians);
		setExstensionExtended(extended);
	}

	/**
	 * Sets the state of the entire arm in a command.
	 * @param shoulderPosRadians The radian value to set the shoulder angle to (between -90 and 30).
	 * @param wristPosRadians The radian value to set the wrist angle to (between -45 and 45).
	 * @param extended Whether the extension is extended.
	 * @return The command to set the state of the arm.
	 */
	public CommandBase setArmStateC(double shoulderPosRadians, double wristPosRadians, boolean extended){
		return run(()->setArmState(shoulderPosRadians, wristPosRadians, extended));
	}

	public CommandBase inC(){
		return sequence(
            setShoulderPosRadiansC(Units.degreesToRadians(-90)),
            setWristPosRadiansC(Units.degreesToRadians(90)),
			setExstensionExtendedC(false)
        );
	}

	public CommandBase scoreMidC(){
		return sequence(
            setShoulderPosRadiansC(Units.degreesToRadians(-7)),
            setWristPosRadiansC(Units.degreesToRadians(7)),
			setExstensionExtendedC(false)
        );
	}

	public CommandBase scoreUpperC(){
		return sequence(
            setShoulderPosRadiansC(Units.degreesToRadians(14)),
            setWristPosRadiansC(Units.degreesToRadians(-14)),
			setExstensionExtendedC(true)
        );
	}

	public CommandBase pickUpGroundC(){
		return sequence(
            setShoulderPosRadiansC(Units.degreesToRadians(-51)),
            setWristPosRadiansC(Units.degreesToRadians(51)),
			setExstensionExtendedC(true)
        );
	}

	public CommandBase pickUpDoubleSubC(){
		return sequence(
            setShoulderPosRadiansC(Units.degreesToRadians(-0.5)),
            setWristPosRadiansC(Units.degreesToRadians(-0.5)),
			setExstensionExtendedC(false)
        );
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

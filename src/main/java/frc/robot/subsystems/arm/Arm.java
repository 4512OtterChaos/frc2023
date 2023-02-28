// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OCSparkMax;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.*;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

@Log.Exclude
public class Arm extends SubsystemBase implements Loggable{
	OCSparkMax shoulderMotor = new OCSparkMax(0, MotorType.kBrushless);
	OCSparkMax wristMotor = new OCSparkMax(3, MotorType.kBrushless);

	private DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(6);
	private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(8);
	
	public ProfiledPIDController shoulderPid = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(0.7, 0.7));
	public ProfiledPIDController wristPid = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(0.8, 0.8));

	DoubleSolenoid extensionPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
	
	private double wristMinimumAngle = kWristMinimumAngle;
    private double wristMaximumAngle = kWristMaximumAngle;
	
	private double shoulderWristLevel = kShoulderWristLevel;
	private double shoulderExstensionWristLevel = kShoulderWristLevel;

    private double shoulderMinimumAngle = kShoulderMinimumAngle;
    private double shoulderMinimumAngleWrist = kShoulderMinimumAngleWrist;
    private double shoulderMinimumAngleExtension = kShoulderMinimumAngleExtension;
    private double shoulderMinimumAngleExtensionWrist = kShoulderMinimumAngleExtensionWrist;
    private double shoulderMaximumAngle = kShoulderMaximumAngle;

	ArmSimulation armSim = new ArmSimulation(shoulderMotor, wristMotor, extensionPiston, shoulderEncoder, wristEncoder);
	
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
		shoulderEncoder.setDistancePerRotation(2 * (Math.PI));
		wristEncoder.setDistancePerRotation(2 * (Math.PI));
		wristMotor.setInverted(false);
		extensionPiston.set(Value.kReverse);
		SmartDashboard.putData("Arm Subsystem", this);
	}

	@Override
	public void periodic() {
		armSim.periodic();
		SmartDashboard.putNumber("Shoulder Angle: Degrees", Units.radiansToDegrees(getShoulderPosRadians()));
		SmartDashboard.putNumber("Shoulder Goal: Degrees", Units.radiansToDegrees(shoulderPid.getGoal().position));
		SmartDashboard.putNumber("Wrist Angle: Degrees", Units.radiansToDegrees(getWristPosRadians()));
		SmartDashboard.putNumber("Wrist Goal: Degrees", Units.radiansToDegrees(wristPid.getGoal().position));
		SmartDashboard.putBoolean("Extension State", getExtensionState());

		

		double shoulderPosRadians = getShoulderPosRadians();
		double wristPosRadians = getWristPosRadians();

		// Calculate shoulder volts using feedforward and PID
		var shoulderSetpoint = shoulderPid.getSetpoint();
		armSim.setShoulderSetpoint(shoulderSetpoint.position);
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
		armSim.setWristSetpoint(wristSetpoint.position+shoulderSetpoint.position);
		double wristGravityVolts = Math.cos(getWristPosFrameRelativeRadians())*wristkg;
		double wristVolts = wristPid.calculate(wristPosRadians) + kWristFF.calculate(wristSetpoint.position, wristSetpoint.velocity) + wristGravityVolts;

		// Clamp wrist position to an appropriate angle.
		double clampedWristPos = wristSafety(wristPosRadians, shoulderPosRadians, getExtensionState());
		// System.out.println("clampedWristPos: " + Units.radiansToDegrees(clampedWristPos));
		// Clamp wrist volts to have the wrist stay inside/go into appropriate angle range.
		if (clampedWristPos > wristPosRadians){
			wristVolts = Math.max(wristGravityVolts + 1, wristVolts);
		}
		if (clampedWristPos < wristPosRadians){
			wristVolts = Math.min(wristGravityVolts + 1, wristVolts);
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

	private double shoulderSetpointSafety(double shoulderPosRadians, boolean extensionState){
		double minimumPosRadians = shoulderMinimumAngle;
		// exstension out
		if (extensionState == true){
			minimumPosRadians = shoulderMinimumAngleExtension;
			
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
		shoulderMotor.setVoltage(volts);
	}

	/**
	 * Sets the position of the shoulder while clamping the angle to stay within a safe range.
	 * @param posRadians The radian value to set the shoulder angle to (between -90 and 30).
	 */
	public void setShoulderPosRadians(double posRadians) {
		double clampedPosRadians = shoulderSetpointSafety(posRadians, getExtensionState());
		shoulderPid.setGoal(clampedPosRadians);
		setWristPosRadians(wristPid.getGoal().position);
	}

	/**
	 * Sets the position of the shoulder in a command.
	 * @param posRadians The radian value to set the shoulder angle to (between -90 and 30).
	 * @return The command to set the position of the shoulder.
	 */
	public CommandBase setShoulderPosRadiansC(double posRadians) {
		return run(() -> setShoulderPosRadians(posRadians)).until(() -> shoulderPid.atGoal()).withTimeout(0.1);
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
		wristMotor.setVoltage(volts);
	}

	/**
	 * Sets the position of the wrist while clamping the angle to stay within a safe range that changes depending on the shoulder angle.
	 * @param posRadians The radian value to set the wrist angle to (between -45 and 45).
	 */
	public void setWristPosRadians(double posRadians) {
		double clampedPosRadians = wristSafety(posRadians, shoulderPid.getGoal().position, getExtensionState());
		wristPid.setGoal(clampedPosRadians);
	}

	/**
	 * Sets the position of the wrist in a command.
	 * @param posRadians The radian value to set the wrist angle to (between -45 and 45).
	 * @return The command to set the position of the wrist.
	 */
	public CommandBase setWristPosRadiansC(double posRadians) {
		return run(() -> setWristPosRadians(posRadians)).until(() -> wristPid.atGoal()).withTimeout(0.1);
	}

	/**
	 * 
	 * @return The wrist position in radians.
	 */
	public double getWristPosRadians() {
		return wristEncoder.getDistance();
	}

	public double getWristPosFrameRelativeRadians() {
		return wristEncoder.getDistance()+getShoulderPosRadians();
	}

	/**
	 * Toggles the extension piston between forward (out) and reverse (in).
	 */
	public void toggleExstensionExtended(){
		if (getExtensionState() == true || getShoulderPosRadians() >= Units.degreesToRadians(-52)){
			 extensionPiston.toggle();
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
		if (exstended==true && getShoulderPosRadians() >= Units.degreesToRadians(-52)){
			extensionPiston.set(Value.kForward);
		}
		else{
			extensionPiston.set(Value.kReverse);
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
		return run(()-> setExstensionExtended(exstended)).withTimeout(0.5);
	}
	
	/**
	 * 
	 * @return Whether the extension is extended.
	 */
	public boolean getExtensionState(){
		if (extensionPiston.get()==Value.kForward){
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

	@Override
	public void simulationPeriodic() {
		armSim.simulationPeriodic();
	}

	public CommandBase inC(){
		return parallel(
			sequence(
				setExstensionExtendedC(false),
				setShoulderPosRadiansC(Units.degreesToRadians(-90)),
				setWristPosRadiansC(Units.degreesToRadians(90))
			),
			waitSeconds(4)
		);
	}

	public CommandBase pickUpGroundC(){
		return parallel(
			sequence(
				setShoulderPosRadiansC(Units.degreesToRadians(-51)),
				setWristPosRadiansC(Units.degreesToRadians(51)),
				waitSeconds(3),
				setExstensionExtendedC(true)
			),
		waitSeconds(5)
		);
	}

	public CommandBase scoreMidC(){
		return parallel(
			sequence(
				setExstensionExtendedC(false),
				setShoulderPosRadiansC(Units.degreesToRadians(-7)),
				setWristPosRadiansC(Units.degreesToRadians(7))
			),
		waitSeconds(4)
	);
	}

	public CommandBase scoreUpperC(){
		return parallel(
			sequence(
				setShoulderPosRadiansC(Units.degreesToRadians(14)),
				setWristPosRadiansC(Units.degreesToRadians(-14)),
				waitSeconds(3),
				setExstensionExtendedC(true)
			),
		waitSeconds(4)
	);
	}

	public CommandBase pickUpDoubleSubC(){
		return parallel(
			sequence(
				setExstensionExtendedC(false),
				setShoulderPosRadiansC(Units.degreesToRadians(-0.5)),
				setWristPosRadiansC(Units.degreesToRadians(-0.5))
			),
		waitSeconds(5)
	);
	}
}

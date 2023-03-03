// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCConfig;
import frc.robot.util.OCSparkMax;
import frc.robot.util.sim.PistonSim;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.*;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

@Log.Exclude
public class Arm extends SubsystemBase implements Loggable {

	private final OCSparkMax shoulderMotorA = new OCSparkMax(8, MotorType.kBrushless);
    private final OCSparkMax shoulderMotorB = new OCSparkMax(9, MotorType.kBrushless);
	private final OCSparkMax wristMotor = new OCSparkMax(10, MotorType.kBrushless);
    private final DoubleSolenoid extensionPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

	private DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(0);
	private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);
	
	@Config.PIDController
	public ProfiledPIDController shoulderPid = new ProfiledPIDController(
            kShoulderKP, kShoulderKI, kShoulderKD,
            new Constraints(Math.toRadians(kShoulderVelocityDeg), Math.toRadians(kShoulderAccelDeg)));
    private double shoulderGoal = 0;
	@Config.PIDController
	public ProfiledPIDController wristPid = new ProfiledPIDController(
            kWristKP, kWristKI, kWristKD,
            new Constraints(Math.toRadians(kWristVelocityDeg), Math.toRadians(kWristAccelDeg)));
    private double wristGroundRelativeGoal = 0;
	
	private double wristMinimumRads = Math.toRadians(kWristMinimumDeg);
    private double wristMaximumRads = Math.toRadians(kWristMaximumDeg);

    private double shoulderMinimumRads = Math.toRadians(kShoulderMinimumDeg);
    private double shoulderMaximumRads = Math.toRadians(kShoulderMaximumDeg);
	
	private double shoulderWristLevel = kShoulderWristLevel;
	private double shoulderExtensionWristLevel = kShoulderWristLevel;
    
    private double shoulderMinimumAngleWrist = kShoulderMinimumAngleWrist;
    private double shoulderMinimumAngleExtension = kShoulderMinimumAngleExtension;
    private double shoulderMinimumAngleExtensionWrist = kShoulderMinimumAngleExtensionWrist;
    
	private final ArmSimulation armSim = new ArmSimulation(
            shoulderMotorA, wristMotor, extensionPiston, shoulderEncoder, wristEncoder);
    private static final Translation2d kRobotSafetyBoxTR =
            new Translation2d(FieldUtil.kRobotLength / 2 + 1, FieldUtil.kRobotBumperHeight + 1);

	public Arm() {
		OCConfig.configMotors(kShoulderStallLimit, kShoulderStallLimit, kRampRate, shoulderMotorA, shoulderMotorB);
        if(RobotBase.isReal()) {
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        }
        shoulderMotorA.setInverted(false);
        shoulderMotorB.follow(shoulderMotorA, true);
		OCConfig.setStatusNothing(shoulderMotorB);
		OCConfig.saveConfig(shoulderMotorA, shoulderMotorB);

        OCConfig.configMotors(kWristStallLimit, kWristStallLimit, kRampRate, wristMotor);
        if(RobotBase.isReal()) {
            OCConfig.setStatusNothing(wristMotor);
            wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 50);
            wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        }
        wristMotor.setInverted(false);
        OCConfig.saveConfig(wristMotor);

        OCConfig.setIdleMode(IdleMode.kBrake, shoulderMotorA, shoulderMotorB, wristMotor);

		shoulderEncoder.setDistancePerRotation(2 * (Math.PI));
		wristEncoder.setDistancePerRotation(2 * (Math.PI));
		
		shoulderPid.setTolerance(Math.toRadians(kShoulderPosToleranceDeg), Math.toRadians(kShoulderVelToleranceDeg));
        wristPid.setTolerance(Math.toRadians(kWristPosToleranceDeg), Math.toRadians(kWristVelToleranceDeg));

        extensionPiston.set(Value.kReverse);

		SmartDashboard.putData("Arm/Subsystem", this);
	}

	@Override
	public void periodic() {
		armSim.periodic();

		double shoulderPosRadians = getShoulderPosRadians();
		double wristPosRadians = getWristPosRadians();
        double extension = armSim.getExtension();

        //TODO periodic goal safety:
        // double clampedShoulderGoal = shoulderSafety2(shoulderGoal, wristGroundRelativeGoal - shoulderGoal, extension);
        // shoulderPid.setGoal(clampedShoulderGoal);
        // wristPid.setGoal(wristGroundRelativeGoal - clampedShoulderGoal);

        shoulderPid.setGoal(shoulderGoal);
        wristPid.setGoal(wristGroundRelativeGoal - shoulderGoal);

		// Calculate shoulder volts:
		var shoulderSetpoint = shoulderPid.getSetpoint();
		armSim.setShoulderSetpoint(shoulderSetpoint.position);
        // update profile setpoint and get PID response voltage,
		double shoulderVolts = shoulderPid.calculate(shoulderPosRadians);
        // and feedforward voltage to follow the profile setpoint.
        shoulderVolts += kShoulderFF.calculate(shoulderSetpoint.position, shoulderSetpoint.velocity);

		// Clamp shoulder position to an appropriate angle.
		double clampedShoulderPos = shoulderSafety(shoulderPosRadians, wristPosRadians, getExtensionState());
        double clampedShoulderGravityVolts = kShoulderkg*Math.cos(clampedShoulderPos);

		// Clamp shoulder volts to have the shoulder stay inside/go into appropriate angle range.
		if (clampedShoulderPos > shoulderPosRadians){
			shoulderVolts = Math.max(clampedShoulderGravityVolts, shoulderVolts);
		}
		if (clampedShoulderPos < shoulderPosRadians){
			shoulderVolts = Math.min(clampedShoulderGravityVolts, shoulderVolts);
		}
        // If encoder is disconnected, make arm go limp
        if(!shoulderEncoder.isConnected()) {
            shoulderVolts = 0;
            setExtension(false);
        }
		// Set shoulder motors to the clamped voltage.
		shoulderMotorA.setVoltage(shoulderVolts);

		// Calculate wrist volts:
		var wristSetpoint = wristPid.getSetpoint();
		armSim.setWristSetpoint(wristSetpoint.position + shoulderSetpoint.position);
        // update profile setpoint and get PID response voltage,
		double wristVolts = wristPid.calculate(wristPosRadians);
        // and feedforward voltage to follow the profile setpoint.
        wristVolts += kWristFF.calculate(
            wristSetpoint.position + shoulderPosRadians, // get the wrist angle relative to the ground
            wristSetpoint.velocity //- shoulderSetpoint.velocity // include shoulder movement
        );

		// Clamp wrist position to an appropriate angle.
		double clampedWristPos = wristSafety(wristPosRadians, shoulderPosRadians, getExtensionState());
        double clampedWristGravityVolts = kWristkg*Math.cos(clampedWristPos);
		// Clamp wrist volts to have the wrist stay inside/go into appropriate angle range.
		if (clampedWristPos > wristPosRadians) {
			wristVolts = Math.max(clampedWristGravityVolts, wristVolts);
		}
		if (clampedWristPos < wristPosRadians) {
			wristVolts = Math.min(clampedWristGravityVolts, wristVolts);
		}
        // If encoder is disconnected, hold wrist level
        if(!wristEncoder.isConnected()) {
            wristVolts = kWristkg;
        }
		// Set wrist motors to the clamped voltage.
		wristMotor.setVoltage(wristVolts);
	}
	
	private double shoulderSafety(double shoulderPosRadians, double wristPosRadians, boolean extensionState){
		double minimumPosRadians = shoulderMinimumRads;
		// extension out
		if (extensionState){
			// if wrist angle is less than level
			if (wristPosRadians < -Units.degreesToRadians(8)){
				minimumPosRadians = shoulderMinimumAngleExtensionWrist;
			}
			else{
				minimumPosRadians = shoulderMinimumAngleExtension;
			}
		}
		// if extension in and wrist is less than level
		if (!extensionState && wristPosRadians < -Units.degreesToRadians(5)){
			minimumPosRadians = shoulderMinimumAngleWrist;
			
		}
		return MathUtil.clamp(shoulderPosRadians, minimumPosRadians, shoulderMaximumRads);
	}
    //TODO periodic goal safety:
    private double shoulderSafety2(double shoulderPosRadians, double wristPosRadians, double extension){
        double minimumPosRadians = shoulderMinimumRads;
        var wristPivotTrl = ArmSimulation.getWristPivotXZ(shoulderPosRadians, extension);
        var wristEndTrl = ArmSimulation.getWristEndXZ(shoulderPosRadians, extension, wristPosRadians);
        if(wristPivotTrl.getY() < 1) {
            minimumPosRadians = Math.max(
                minimumPosRadians,
                ArmSimulation.getShoulderRadsMovedY(wristPivotTrl, 1 - wristPivotTrl.getY()).orElse(minimumPosRadians)
            );
        }
        if(wristEndTrl.getY() < 1) {
            minimumPosRadians = Math.max(
                minimumPosRadians,
                ArmSimulation.getShoulderRadsMovedY(wristEndTrl, 1 - wristEndTrl.getY()).orElse(minimumPosRadians)
            );
        }
        if(wristPivotTrl.getX() < 0) {
            minimumPosRadians = Math.max(
                minimumPosRadians,
                ArmSimulation.getShoulderRadsMovedX(wristPivotTrl, 0 - wristPivotTrl.getX()).orElse(minimumPosRadians)
            );
        }
        if(wristEndTrl.getX() < 0) {
            minimumPosRadians = Math.max(
                minimumPosRadians,
                ArmSimulation.getShoulderRadsMovedX(wristEndTrl, 0 - wristEndTrl.getX()).orElse(minimumPosRadians)
            );
        }
        if(wristPivotTrl.getX() < kRobotSafetyBoxTR.getX() && wristPivotTrl.getY() < kRobotSafetyBoxTR.getY()) {
            double movedX = Math.max(
                minimumPosRadians,
                ArmSimulation.getShoulderRadsMovedX(wristEndTrl, kRobotSafetyBoxTR.getX() - wristEndTrl.getX())
                    .orElse(minimumPosRadians)
            );
            double movedY = Math.max(
                minimumPosRadians,
                ArmSimulation.getShoulderRadsMovedY(wristEndTrl, kRobotSafetyBoxTR.getY() - wristEndTrl.getY())
                    .orElse(minimumPosRadians)
            );
            if(movedX - minimumPosRadians < movedY - minimumPosRadians && movedX != minimumPosRadians) {
                minimumPosRadians = movedX;
            }
            else {
                minimumPosRadians = movedY;
            }
        }
        return MathUtil.clamp(shoulderPosRadians, minimumPosRadians, shoulderMaximumRads);
    }

	private double shoulderSetpointSafety(double shoulderPosRadians, boolean extensionState){
		double minimumPosRadians = shoulderMinimumRads;
		// extension out
		if (extensionState){
			minimumPosRadians = shoulderMinimumAngleExtension;
			
		}
		return MathUtil.clamp(shoulderPosRadians, minimumPosRadians, shoulderMaximumRads);
	}

	private double wristSafety(double wristPosRadians, double shoulderPosRadians, boolean extensionState){
		double minimumPosRadians = wristMinimumRads;
		//extension out and shoulder goal is below set angle
		if (extensionState == true && shoulderPid.getGoal().position < shoulderExtensionWristLevel){
			minimumPosRadians = -shoulderPid.getGoal().position;
		}
		//extension in and shoulder goal is below set angle
		if (extensionState == false && shoulderPid.getGoal().position < shoulderWristLevel){
			minimumPosRadians = -shoulderPid.getGoal().position;
		}
		
		return MathUtil.clamp(wristPosRadians, minimumPosRadians, wristMaximumRads);
	}

	/**
	 * Sets the position of the shoulder while clamping the angle to stay within a safe range.
	 * @param posRadians The radian value to set the shoulder angle to (between -90 and 30).
	 */
	public void setShoulderPosRadians(double posRadians) {
        //TODO periodic goal safety:
		// double clampedPosRadians = shoulderSetpointSafety(posRadians, getExtensionState());
		// shoulderPid.setGoal(clampedPosRadians);
		// setWristPosRadians(wristPid.getGoal().position);

        shoulderGoal = posRadians;
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
		return shoulderEncoder.getDistance() - kShoulderAngleOffset.getRadians();
	}

	/**
	 * Sets the position of the wrist while clamping the angle to stay within a safe range that changes depending on the shoulder angle.
	 * @param posRadians The radian value to set the wrist angle to (between -30 and 90).
	 */
	public void setWristPosGroundRelRads(double posRadians) {
        //TODO periodic goal safety:
		// double clampedPosRadians = wristSafety(posRadians, shoulderPid.getGoal().position, getExtensionState());
		// wristPid.setGoal(clampedPosRadians);

        wristGroundRelativeGoal = posRadians;
	}

	/**
	 * Sets the position of the wrist in a command.
	 * @param posRadians The radian value to set the wrist angle to (between -30 and 90).
	 * @return The command to set the position of the wrist.
	 */
	public CommandBase setWristPosRadiansC(double posRadians) {
		return run(() -> setWristPosGroundRelRads(posRadians)).until(() -> wristPid.atGoal());
	}

	/**
	 * @return The wrist position in radians.
	 */
	public double getWristPosRadians() {
		return wristEncoder.getDistance() - kWristAngleOffset.getRadians();
	}

    /**
     * @return The wrist position in radians, relative to the ground.
     */
	public double getWristPosGroundRelativeRadians() {
		return getWristPosRadians()+getShoulderPosRadians();
	}

	/**
	 * Toggles the extension piston between forward (out) and reverse (in).
	 */
	public void toggleExtension(){
		if (getExtensionState() == true || getShoulderPosRadians() >= Units.degreesToRadians(-52)){
			 extensionPiston.toggle();
		}
        //TODO periodic goal safety:
		// setShoulderPosRadians(shoulderPid.getGoal().position);
		// setWristPosGroundRelRads(wristPid.getGoal().position);
	}

	/**
	 * Toggles the extension piston between forward (out) and reverse (in) in a command.
	 * @return The command to toggle the extension piston.
	 */
	public CommandBase toggleExtensionC(){
		return runOnce(()-> toggleExtension());
	}
	
	/**
	 * Sets the state of the extension pistion to the desired state.
	 * @param extensionState The value to set the extension pistion to (forward, off or reverse).
	 */
	public void setExtension(boolean extended){
		if (extended && getShoulderPosRadians() >= Units.degreesToRadians(-52)){
			extensionPiston.set(Value.kForward);
		}
		else{
			extensionPiston.set(Value.kReverse);
		}
        //TODO periodic goal safety:
		// setShoulderPosRadians(shoulderPid.getGoal().position);
		// setWristPosGroundRelRads(wristPid.getGoal().position);
	}

	/**
	 * Sets the state of the extension pistion to the desired state in a command.
	 * @param extended The value to set the extension pistion to (forward, off or reverse).
	 * @return The command to set the extension pistion.
	 */
	public CommandBase setExtensionC(boolean extended){
		return run(()-> setExtension(extended))
			.until(() -> extended ? armSim.getExtension() == 1 : armSim.getExtension() == 0);
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
	 * @param wristPosRadians The radian value to set the wrist angle to (between -30 and 90).
	 * @param extended The value to set the extension pistion to (forward, off or reverse).
	 */
	public void setArmState(double shoulderPosRadians, double wristPosRadians, boolean extended){
		setShoulderPosRadians(shoulderPosRadians);
		setWristPosGroundRelRads(wristPosRadians);
		setExtension(extended);
	}

	/**
	 * Sets the state of the entire arm in a command.
	 * @param shoulderPosRadians The radian value to set the shoulder angle to (between -90 and 30).
	 * @param wristPosRadians The radian value to set the wrist angle to (between -30 and 90).
	 * @param extended Whether the extension is extended.
	 * @return The command to set the state of the arm.
	 */
	public CommandBase setArmStateC(double shoulderPosRadians, double wristPosRadians, boolean extended){
		return run(()->setArmState(shoulderPosRadians, wristPosRadians, extended))
			.until(()->{
				return shoulderPid.atGoal() && wristPid.atGoal() && extended ? armSim.getExtension() == 1 : armSim.getExtension() == 0;
			});
	}

	@Override
	public void simulationPeriodic() {
		armSim.simulationPeriodic();
	}

	public CommandBase inC(){
		return sequence(
			setExtensionC(false),
			setArmStateC(Units.degreesToRadians(-90), Units.degreesToRadians(0), false)
		);
	}

	public CommandBase pickUpGroundC(){
		return sequence(
			setArmStateC(Units.degreesToRadians(-51), Units.degreesToRadians(0), getExtensionState()),
			setExtensionC(true)
		);
	}

	public CommandBase scoreMidC(){
		return sequence(
			setExtensionC(false),
			setArmStateC(Units.degreesToRadians(-7), Units.degreesToRadians(0), false)
		);
	}

	public CommandBase scoreUpperC(){
		return sequence(
			setArmStateC(Units.degreesToRadians(14), Units.degreesToRadians(0), getExtensionState()),
			setExtensionC(true)
		);
	}

	public CommandBase pickUpDoubleSubC(){
		return sequence(
			setExtensionC(false),
			setArmStateC(Units.degreesToRadians(-0.5), Units.degreesToRadians(-1), false)
		);
	}

    public void log() {
        SmartDashboard.putNumber("Arm/Shoulder Degrees", Units.radiansToDegrees(getShoulderPosRadians()));
        SmartDashboard.putNumber("Arm/Shoulder Setpoint Degrees", Units.radiansToDegrees(shoulderPid.getSetpoint().position));
		SmartDashboard.putNumber("Arm/Shoulder Goal Degrees", Units.radiansToDegrees(shoulderPid.getGoal().position));
		SmartDashboard.putNumber("Arm/Wrist Ground Relative Deg", Math.toDegrees(getWristPosGroundRelativeRadians()));
        SmartDashboard.putNumber("Arm/Wrist Degrees", Units.radiansToDegrees(getWristPosRadians()));
        SmartDashboard.putNumber("Arm/Wrist Goal Degrees", Units.radiansToDegrees(wristPid.getSetpoint().position));
		SmartDashboard.putNumber("Arm/Wrist Goal Degrees", Units.radiansToDegrees(wristPid.getGoal().position));
		SmartDashboard.putBoolean("Arm/Extension State", getExtensionState());
    }


    //----- Dashboard configuration
    @Config(defaultValueNumeric = kShoulderMinimumDeg)
	void configShoulderMinimumDeg(double shoulderMinimumDeg) {
		this.shoulderMinimumRads = Math.toRadians(shoulderMinimumDeg);
	}
    @Config(defaultValueNumeric = kShoulderMaximumDeg)
	void configShoulderMaximumDeg(double shoulderMaximumDeg) {
		this.shoulderMaximumRads = Math.toRadians(shoulderMaximumDeg);
	}
    @Config(defaultValueNumeric = kWristMinimumDeg)
	void configWristMinimumDeg(double wristMinimumDeg) {
		this.wristMinimumRads = Math.toRadians(wristMinimumDeg);
	}
	@Config(defaultValueNumeric = kWristMaximumDeg)
	void configWristMaximumDeg(double wristMaximumDeg) {
		this.wristMaximumRads = Math.toRadians(wristMaximumDeg);
	}

	@Config(defaultValueNumeric = kShoulderWristLevel)
	void configShoulderWristLevel(double shoulderWristLevel) {
		this.shoulderWristLevel = shoulderWristLevel;
	}
	@Config(defaultValueNumeric = kShoulderExtensionWristLevel)
	void configShoulderextensionWristLevel(double shoulderextensionWristLevel) {
		this.shoulderExtensionWristLevel = shoulderextensionWristLevel;
	}
	@Config(defaultValueNumeric = kShoulderMinimumAngleWrist)
	void configShoulderMinimumAngleWrist(double shoulderMinimumAngleWrist) {
		this.shoulderMinimumAngleWrist = shoulderMinimumAngleWrist;
	}
	
	@Config(defaultValueNumeric = kShoulderMinimumAngleExtension)
	void configShoulderMinimumAngleExtension(double shoulderMinimumAngleExtension) {
		this.shoulderMinimumAngleExtension = shoulderMinimumAngleExtension;
	}
	@Config(defaultValueNumeric = kShoulderMinimumAngleExtensionWrist)
	void configShoulderMinimumAngleExtensionWrist(double shoulderMinimumAngleExtensionWrist) {
		this.shoulderMinimumAngleExtensionWrist = shoulderMinimumAngleExtensionWrist;
	}
	double shoulderks = kShoulderks;
	double shoulderkg = kShoulderkg;
	double shoulderkv = kShoulderkv;
	double shoulderka = kShoulderka;
	@Config(defaultValueNumeric = kShoulderks)
	void configShoulderks(double ks) {
		shoulderks = ks;
		kShoulderFF = new ArmFeedforward(ks, shoulderkg, shoulderkv, shoulderka);
	}
	@Config(defaultValueNumeric = kShoulderkg)
	void configShoulderkg(double kg) {
		shoulderkg = kg;
		kShoulderFF = new ArmFeedforward(shoulderks, kg, shoulderkv, shoulderka);
	}
	@Config(defaultValueNumeric = kShoulderkv)
	void configShoulderkv(double kv) {
		shoulderkv = kv;
		kShoulderFF = new ArmFeedforward(shoulderks, shoulderkg, kv, shoulderka);
	}
	@Config(defaultValueNumeric = kShoulderka)
	void configShoulderka(double ka) {
		shoulderka = ka;
		kShoulderFF = new ArmFeedforward(shoulderks, shoulderkg, shoulderkv, ka);
	}
	double wristks = kWristks;
	double wristkg = kWristkg;
	double wristkv = kWristkv;
	double wristka = kWristka;
	@Config(defaultValueNumeric = kWristks)
	void configWristks(double ks) {
		wristks = ks;
		kWristFF = new ArmFeedforward(ks, wristkg, wristkv, wristka);
	}
	@Config(defaultValueNumeric = kWristkg)
	void configWristkg(double kg) {
		wristkg = kg;
		kWristFF = new ArmFeedforward(wristks, kg, wristkv, wristka);
	}
	@Config(defaultValueNumeric = kWristkv)
	void configWristkv(double kv) {
		wristkv = kv;
		kWristFF = new ArmFeedforward(wristks, wristkg, kv, wristka);
	}
	@Config(defaultValueNumeric = kWristka)
	void configWristka(double ka) {
		wristka = ka;
		kWristFF = new ArmFeedforward(wristks, wristkg, wristkv, ka);
	}
	double shoulderConfigPIDVelocityConstraint = kShoulderVelocityDeg;
	double shoulderConfigPIDAccelerationConstraint = kShoulderAccelDeg;
	@Config(defaultValueNumeric = kShoulderVelocityDeg)
	void configShoulderPIDVelocityConstraint(double velocity) {
		shoulderConfigPIDVelocityConstraint = velocity;
		shoulderPid.setConstraints(new Constraints(Math.toRadians(shoulderConfigPIDVelocityConstraint), Math.toRadians(shoulderConfigPIDAccelerationConstraint)));
	}
	@Config(defaultValueNumeric = kShoulderAccelDeg)
	void configShoulderPIDAccelerationConstraint(double acceleration) {
		shoulderConfigPIDAccelerationConstraint = acceleration;
		shoulderPid.setConstraints(new Constraints(Math.toRadians(shoulderConfigPIDVelocityConstraint), Math.toRadians(shoulderConfigPIDAccelerationConstraint)));
	}
	double wristConfigPIDVelocityConstraint = kWristVelocityDeg;
	double wristConfigPIDAccelerationConstraint = kWristAccelDeg;
	@Config(defaultValueNumeric = kWristVelocityDeg)
	void configWristPIDVelocityConstraint(double velocity) {
		wristConfigPIDVelocityConstraint = velocity;
		wristPid.setConstraints(new Constraints(Math.toRadians(velocity), Math.toRadians(shoulderConfigPIDAccelerationConstraint)));
	}
	@Config(defaultValueNumeric = kWristAccelDeg)
	void configWristPIDAccelerationConstraint(double acceleration) {
		wristConfigPIDAccelerationConstraint = acceleration;
		wristPid.setConstraints(new Constraints(Math.toRadians(shoulderConfigPIDVelocityConstraint), Math.toRadians(acceleration)));
	}
}

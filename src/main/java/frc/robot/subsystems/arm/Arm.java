// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ArmLerpFeedforward;
import frc.robot.util.FieldUtil;
import frc.robot.util.MathHelp;
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

	private final OCSparkMax shoulderMotorA = new OCSparkMax(5, MotorType.kBrushless);
    private final OCSparkMax shoulderMotorB = new OCSparkMax(13, MotorType.kBrushless);
	private final OCSparkMax wristMotor = new OCSparkMax(12, MotorType.kBrushless);
    private final DoubleSolenoid extensionPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 15, 0);

	private DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(9);
	private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(7);
	
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
	
	private double wristMinimum = Math.toRadians(kWristMinimumDeg);
    private double wristMaximum = Math.toRadians(kWristMaximumDeg);

    private double shoulderMinimum = Math.toRadians(kShoulderMinimumDeg);
    private double shoulderMaximum = Math.toRadians(kShoulderMaximumDeg);
	
    private double shoulderMinWristDown = Math.toRadians(kShoulderMinWristDownDeg);
    private double shoulderMinExtended = Math.toRadians(kShoulderMinExtendedDeg);
    private double shoulderMinExtendedWristDown = Math.toRadians(kShoulderMinExtendedWristDownDeg);

    private boolean isOuttakingCone = false;
    private boolean isOuttakingHigh = false;
    
    private final PistonSim pistonSim = new PistonSim(false, 0.5);
	private final ArmSimulation armSim = new ArmSimulation(
            this, shoulderMotorA, wristMotor, shoulderEncoder, wristEncoder);
    private static final Translation2d kRobotSafetyBoxTR =
            new Translation2d(FieldUtil.kRobotLength / 2 + 1, FieldUtil.kRobotBumperHeight + 1);


	public double shoulderTestVolts = 0;
	public double wristTestVolts = 0;

	private double setShoulderTime = Timer.getFPGATimestamp();
	private double setWristTime = Timer.getFPGATimestamp();
	private double setPistonTime = Timer.getFPGATimestamp();

	public Arm() {
        shoulderMotorA.setCANTimeout(100);
        shoulderMotorB.setCANTimeout(100);
        wristMotor.setCANTimeout(100);
		OCConfig.configMotors(kShoulderStallLimit, kShoulderStallLimit, kRampRate, shoulderMotorA, shoulderMotorB);
		Timer.delay(0.5);
        if(RobotBase.isReal()) {
            
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
            shoulderMotorA.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
			Timer.delay(0.5);
            shoulderMotorA.setInverted(true);
			// shoulderMotorB.setInverted(false);
			Timer.delay(0.5);
            shoulderMotorB.follow(shoulderMotorA, true);
        }
        
		// OCConfig.setStatusNothing(shoulderMotorB);
		OCConfig.setStatusSlow(shoulderMotorB);
        OCConfig.setIdleMode(IdleMode.kBrake, shoulderMotorA, shoulderMotorB);
        shoulderMotorA.setCANTimeout(0);
        shoulderMotorB.setCANTimeout(0);
        
		OCConfig.saveConfig(shoulderMotorA, shoulderMotorB);

        OCConfig.configMotors(kWristStallLimit, kWristStallLimit, kRampRate, wristMotor);
        if(RobotBase.isReal()) {
            OCConfig.setStatusNothing(wristMotor);
            wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
            wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);

            wristMotor.setInverted(false);
        }
        
        OCConfig.setIdleMode(IdleMode.kBrake, wristMotor);
        wristMotor.setCANTimeout(0);
        OCConfig.saveConfig(wristMotor);

        double radsPerRot = 2 * Math.PI;
		shoulderEncoder.setDistancePerRotation(RobotBase.isReal() ? -radsPerRot : radsPerRot);
		wristEncoder.setDistancePerRotation(radsPerRot);
		
		shoulderPid.setTolerance(Math.toRadians(kShoulderPosToleranceDeg), Math.toRadians(kShoulderVelToleranceDeg));
        wristPid.setTolerance(Math.toRadians(kWristPosToleranceDeg), Math.toRadians(kWristVelToleranceDeg));

        // extensionPiston.set(Value.kReverse);

		SmartDashboard.putData("Arm/Subsystem", this);
		setShoulderWristExt(shoulderMinimum, 0, false);
	}

	public void init(boolean auto){
		shoulderPid.reset(getShoulderPosRadians());
		wristPid.reset(getWristPosRadians());
	}

	@Override
	public void periodic() {
		double shoulderPos = getShoulderPosRadians();
		double wristPos = getWristPosRadians();
        boolean extended = getExtended();
        pistonSim.setExtended(extended);
        pistonSim.update();
        double extension = getExtension();

        // visualize the current arm state to a Mechanism2d
        armSim.visualizeState(shoulderPos, wristPos, extended);

        // shoulderPid.setGoal(shoulderGoal);
        // wristPid.setGoal(wristGroundRelativeGoal - shoulderGoal);

        //TODO periodic goal safety:
        double wristGoal = wristGroundRelativeGoal - shoulderGoal;
        double clampedShoulderGoal = shoulderSafety(shoulderGoal, wristGoal, extended);
        wristGoal = wristGroundRelativeGoal - clampedShoulderGoal;
        // double clampedShoulderGoal = shoulderSafety2(shoulderGoal, wristGroundRelativeGoal - shoulderGoal, extension);
        double clampedWristGoal = wristSafety(wristGoal, clampedShoulderGoal, extended);
        // shoulderPid.setGoal(clampedShoulderGoal);
        // wristPid.setGoal(clampedWristGoal);

		// Calculate shoulder volts:
		var shoulderSetpoint = shoulderPid.getSetpoint();
        // update profile setpoint and get PID response voltage,
		double shoulderVolts = shoulderPid.calculate(shoulderPos);
        // and feedforward voltage to follow the profile setpoint.
        shoulderVolts += kShoulderFF.calculate(shoulderPos, shoulderSetpoint.velocity, extension);

		shoulderVolts += shoulderTestVolts;

		// Clamp shoulder position to an appropriate angle.
		double clampedShoulderPos = shoulderSafety(shoulderPos, wristPos, getExtended());
        double clampedShoulderGravityVolts = kShoulderkg1*Math.cos(clampedShoulderPos);
		
		// Clamp shoulder volts to have the shoulder stay inside/go into appropriate angle range.
		if (clampedShoulderPos > shoulderPos){
			shoulderVolts = Math.max(clampedShoulderGravityVolts, shoulderVolts);
		}
		if (clampedShoulderPos < shoulderPos){
			shoulderVolts = Math.min(clampedShoulderGravityVolts, shoulderVolts);
		}
        // If encoder is disconnected, make arm go limp
        if(!shoulderEncoder.isConnected()) {
            shoulderVolts = 0;
            setExtended(false);
            DriverStation.reportError("Shoulder encoder disconnected!", false);
        }
		// Set shoulder motors to the clamped voltage.
		shoulderMotorA.setVoltage(shoulderVolts);
		// shoulderMotorA.setVoltage(0);

		// Calculate wrist volts:
		var wristSetpoint = wristPid.getSetpoint();
        // update profile setpoint and get PID response voltage,
		double wristVolts = wristPid.calculate(wristPos);
        // and feedforward voltage to follow the profile setpoint.
        wristVolts += kWristFF.calculate(
            wristPos + shoulderPos, // get the wrist angle relative to the ground
            wristSetpoint.velocity //- shoulderSetpoint.velocity // include shoulder movement
        );

		wristVolts += wristTestVolts;

		// Clamp wrist position to an appropriate angle.
		double clampedWristPos = wristSafety(wristPos, shoulderPos, getExtended());
        double clampedWristGravityVolts = kWristkg*Math.cos(clampedWristPos);
		// Clamp wrist volts to have the wrist stay inside/go into appropriate angle range.
		if (clampedWristPos > wristPos) {
			wristVolts = Math.max(clampedWristGravityVolts, wristVolts);
		}
		if (clampedWristPos < wristPos) {
			wristVolts = Math.min(clampedWristGravityVolts, wristVolts);
		}
        // If encoder is disconnected, hold wrist level
        if(!wristEncoder.isConnected()) {
            wristVolts = kWristkg;
            DriverStation.reportError("Wrist encoder disconnected!", false);
        }
		// Set wrist motors to the clamped voltage.
		wristMotor.setVoltage(wristVolts);
		// wristMotor.setVoltage(0);

        // visualize the desired arm state to a Mechanism2d
        armSim.visualizeSetpoint(shoulderSetpoint.position, wristSetpoint.position, extended);
	}
	
	private double shoulderSafety(double shoulderPos, double wristPos, boolean extended){
		double minimumPosRadians = shoulderMinimum;
        double maximumPosRadians = shoulderMaximum;
		// if extended and near ground,
		if (extended && shoulderPos <= shoulderMinExtendedWristDown){
            minimumPosRadians = MathHelp.map(
                wristPos, wristMinimum, -shoulderMinExtended, // between wrist fully down and level with ground,
                shoulderMinExtendedWristDown, shoulderMinExtended // ensure shoulder doesnt let it touch the ground
            );
		}
		// if retracted and wrist is not fully up,
		if(!extended && wristPos < Math.toRadians(68)){
            // and stowed: dont go up unless wrist is up
            if(shoulderPos < Math.toRadians(-80)) {
                maximumPosRadians = shoulderMinimum;
            }
            // and not stowed: dont go down unless wrist is up
            else minimumPosRadians = shoulderMinWristDown;
		}
		return MathUtil.clamp(shoulderPos, minimumPosRadians, maximumPosRadians);
	}
    //TODO periodic goal safety:
    private double shoulderSafety2(double shoulderPosRadians, double wristPosRadians, double extension){
        double minimumPosRadians = shoulderMinimum;
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
        return MathUtil.clamp(shoulderPosRadians, minimumPosRadians, shoulderMaximum);
    }

	// private double shoulderSetpointSafety(double shoulderPosRadians, boolean extensionState){
	// 	double minimumPosRadians = shoulderMinimum;
	// 	// extension out
	// 	if (extensionState){
	// 		minimumPosRadians = shoulderMinimumAngleExtension;
			
	// 	}
	// 	return MathUtil.clamp(shoulderPosRadians, minimumPosRadians, shoulderMaximum);
	// }

	private double wristSafety(double wristPos, double shoulderPos, boolean extended){
		double minimumPosRadians = wristMinimum;
        double maximumPosRadians = wristMaximum;

        // if extended and near ground,
		if (extended && shoulderPos <= shoulderMinExtendedWristDown){
            minimumPosRadians = MathHelp.map(
                shoulderPos, shoulderMinExtendedWristDown, shoulderMinExtended, // based on shoulder angle,
                wristMinimum, -shoulderMinExtended // ensure wrist doesnt touch the ground
            );
		}
        // if retracted and stowing,
        if(!extended && shoulderPos <= shoulderMinWristDown) {
            // ensure wrist is fully up
            minimumPosRadians = Math.toRadians(89.5);
        }
		
		return MathUtil.clamp(wristPos, minimumPosRadians, maximumPosRadians);
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
		// setShoulderTime = Timer.getFPGATimestamp();
		shoulderPid.setGoal(shoulderGoal);
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
        Rotation2d shoulderPos = new Rotation2d(shoulderEncoder.getDistance());
        if(RobotBase.isSimulation()){ return shoulderPos.getRadians();}
        else return shoulderPos.minus(kShoulderAngleOffset).getRadians();
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
		// setWristTime = Timer.getFPGATimestamp();
		wristPid.setGoal(wristGroundRelativeGoal - shoulderGoal);
	}

	/**
	 * Sets the position of the wrist in a command.
	 * @param posRadians The radian value to set the wrist angle to (between -30 and 90).
	 * @return The command to set the position of the wrist.
	 */
	public CommandBase setWristPosRadiansC(double posRadians) {
		return run(() -> setWristPosGroundRelRads(getShoulderPosRadians()+posRadians)).until(() -> wristPid.atGoal());
	}

	/**
	 * @return The wrist position in radians.
	 */
	public double getWristPosRadians() {
		Rotation2d wristPos = new Rotation2d(wristEncoder.getDistance());
        if(RobotBase.isSimulation()){ return wristPos.getRadians();}
        else return wristPos.minus(kWristAngleOffset).getRadians();
	}

    /**
     * @return The wrist position in radians, relative to the ground.
     */
	public double getWristGroundRelPosRadians() {
		return new Rotation2d(getWristPosRadians()+getShoulderPosRadians()).getRadians();
	}

    public boolean getAtShoulderGoal() {
        return Math.abs(getShoulderPosRadians() - shoulderGoal) < shoulderPid.getPositionTolerance()
                && Math.abs(shoulderPid.getVelocityError()) < shoulderPid.getVelocityTolerance();
    }
    public boolean getAtWristGoal() {
        return Math.abs(getWristGroundRelPosRadians() - wristGroundRelativeGoal) < wristPid.getPositionTolerance()
                && Math.abs(wristPid.getVelocityError()) < wristPid.getVelocityTolerance();
    }

	// /**
	//  * Toggles the extension piston between forward (out) and reverse (in).
	//  */
	// public void toggleExtended(){
	// 	if (getExtended() || getShoulderPosRadians() >= Math.toRadians(-52)){
	// 		 extensionPiston.toggle();
	// 	}
    //     //TODO periodic goal safety:
	// 	// setShoulderPosRadians(shoulderPid.getGoal().position);
	// 	// setWristPosGroundRelRads(wristPid.getGoal().position);
	// }

	// /**
	//  * Toggles the extension piston between forward (out) and reverse (in) in a command.
	//  * @return The command to toggle the extension piston.
	//  */
	// public CommandBase toggleExtendedC(){
	// 	return runOnce(()-> toggleExtended());
	// }
	
	/**
	 * Sets the state of the extension pistion to the desired state.
	 * @param extensionState The value to set the extension pistion to (forward, off or reverse).
	 */
	public void setExtended(boolean extended){
		if (extended && getShoulderPosRadians() >= shoulderMinExtended){
			extensionPiston.set(Value.kForward);
		}
		else{
			extensionPiston.set(Value.kReverse);
		}
		setPistonTime = Timer.getFPGATimestamp();
        //TODO periodic goal safety:
		// setShoulderPosRadians(shoulderPid.getGoal().position);
		// setWristPosGroundRelRads(wristPid.getGoal().position);
	}

	/**
	 * Sets the state of the extension pistion to the desired state in a command.
	 * @param extended The value to set the extension pistion to (forward, off or reverse).
	 * @return The command to set the extension pistion.
	 */
	public CommandBase setExtendedC(boolean extended){
		return run(()-> setExtended(extended))
			.until(() -> extended ? getExtension() == 1 : getExtension() == 0);
	}
	
	/**
	 * 
	 * @return Whether the extension is extended.
	 */
	public boolean getExtended(){
		return extensionPiston.get()==Value.kForward;
	}
    public double getExtension() {
        return pistonSim.getExtension();
    }

    /**
	 * Sets the state of the entire arm.
	 * @param shoulderPos The radian value to set the shoulder angle to.
	 * @param wristGroundRelPos The radian value to set the wrist angle to.
	 * @param extended The value to set the extension pistion to (forward, off or reverse).
	 */
	public void setShoulderWristExt(double shoulderPos, double wristGroundRelPos, boolean extended){
		setShoulderPosRadians(shoulderPos);
		setWristPosGroundRelRads(wristGroundRelPos);
		setExtended(extended);
	}
    /**
	 * Sets the state of the entire arm.
	 * @param shoulderPos The radian value to set the shoulder angle to.
	 * @param wristGroundRelPos The radian value to set the wrist angle to.
	 */
	public void setShoulderWrist(double shoulderPos, double wristGroundRelPos){
		setShoulderPosRadians(shoulderPos);
		setWristPosGroundRelRads(wristGroundRelPos);
	}
    /**
	 * Sets the state of the entire arm.
	 * @param shoulderPos The radian value to set the shoulder angle to.
	 * @param extended The value to set the extension pistion to (forward, off or reverse).
	 */
	public void setShoulderExt(double shoulderPos, boolean extended){
		setShoulderPosRadians(shoulderPos);
		setExtended(extended);
	}
    /**
	 * Sets the state of the entire arm.
	 * @param wristGroundRelPos The radian value to set the wrist angle to.
	 * @param extended The value to set the extension pistion to (forward, off or reverse).
	 */
	public void setWristExt(double wristGroundRelPos, boolean extended){
		setWristPosGroundRelRads(wristGroundRelPos);
		setExtended(extended);
	}

	public void setIsOuttakingCone(boolean isOuttakingCone) {
        this.isOuttakingCone = isOuttakingCone;
    }

    public void setIsOuttakingHigh(boolean isOuttakingHigh) {
        this.isOuttakingHigh = isOuttakingHigh;
    }

    public boolean getIsOuttakingCone() {
        return isOuttakingCone;
    }

    public boolean getIsOuttakingHigh() {
        return isOuttakingHigh;
    }
	
    /**
	 * Sets the state of the entire arm in a command.
	 * @param shoulderPos The radian value to set the shoulder angle to.
	 * @param wristGroundRelPos The radian value to set the wrist angle to.
	 * @param extended Whether the extension is extended.
	 * @return The command to set the state of the arm.
	 */
	public CommandBase setShoulderWristExtC(double shoulderPos, double wristGroundRelPos, boolean extended){
		return run(()->setShoulderWristExt(shoulderPos, wristGroundRelPos, extended))
			.until(()->{
				return getAtShoulderGoal() && getAtWristGoal() && (extended ? getExtension() == 1 : getExtension() == 0);
			});
	}
    /**
	 * Sets the state of the entire arm in a command.
	 * @param shoulderPos The radian value to set the shoulder angle to.
	 * @param wristGroundRelPos The radian value to set the wrist angle to.
	 * @return The command to set the state of the arm.
	 */
	public CommandBase setShoulderWristC(double shoulderPos, double wristGroundRelPos){
		return run(()->setShoulderWrist(shoulderPos, wristGroundRelPos))
			.until(()->{
				return getAtShoulderGoal() && getAtWristGoal();
			});
	}
    /**
	 * Sets the state of the entire arm in a command.
	 * @param shoulderPos The radian value to set the shoulder angle to.
	 * @param extended Whether the extension is extended.
	 * @return The command to set the state of the arm.
	 */
	public CommandBase setShoulderExtC(double shoulderPos, boolean extended){
		return run(()->setShoulderExt(shoulderPos, extended))
			.until(()->{
				return getAtShoulderGoal() && extended ? getExtension() == 1 : getExtension() == 0;
			});
	}
    /**
	 * Sets the state of the entire arm in a command.
	 * @param wristGroundRelPos The radian value to set the wrist angle to.
	 * @param extended Whether the extension is extended.
	 * @return The command to set the state of the arm.
	 */
	public CommandBase setWristExtC(double wristGroundRelPos, boolean extended){
		return run(()->setWristExt(wristGroundRelPos, extended))
			.until(()->{
				return getAtWristGoal() && extended ? getExtension() == 1 : getExtension() == 0;
			});
	}	

	public CommandBase inC(){
		return sequence(
			setExtendedC(false),
            setWristPosRadiansC(wristMaximum).withTimeout(0.5),
			setShoulderPosRadiansC(shoulderMinimum)
		);
	}
	public CommandBase coneInC(){
		return sequence(
			runOnce(()->setIsOuttakingCone(false)),
			setExtendedC(false),
            setWristPosRadiansC(wristMaximum).withTimeout(0.5),
			setShoulderWristExtC(Math.toRadians(-69), wristMaximum, false)
		);
	}

	public CommandBase pickUpGroundC(){
		return sequence(
			either(sequence(
				setWristPosRadiansC(wristMaximum).withTimeout(1.5),
				setShoulderPosRadiansC(Math.toRadians(-65))
			), 
			none(), 
			()->getShoulderPosRadians()<Math.toRadians(kShoulderMinWristDownDeg)),
			setShoulderWristExtC(Math.toRadians(-46), Math.toRadians(-5), true)
		);
	}

	public CommandBase pickUpCubeGroundC(){
		return sequence(
			either(sequence(
				setWristPosRadiansC(wristMaximum).withTimeout(1.5),
				setShoulderPosRadiansC(Math.toRadians(-60)),
				waitSeconds(0.25)
			), 
			none(), 
			()->getShoulderPosRadians()<Math.toRadians(kShoulderMinWristDownDeg)),
			setShoulderWristExtC(Math.toRadians(-53), Math.toRadians(-62), false)
		);
	}

	public CommandBase scoreMidConeC(){
		return sequence(
			runOnce(()->setIsOuttakingCone(true)),
			runOnce(()->setIsOuttakingHigh(false)),
			setExtendedC(false),
			either(sequence(
				setWristPosRadiansC(wristMaximum).withTimeout(1.5),
				setShoulderPosRadiansC(Math.toRadians(-65))
			), 
			none(), 
			()->getShoulderPosRadians()<Math.toRadians(kShoulderMinWristDownDeg)),
			setShoulderWristExtC(Math.toRadians(-6.5), Math.toRadians(48.5), false)
		);
	}

	public CommandBase scoreHighConeC(){
		return sequence(
			runOnce(()->setIsOuttakingCone(true)),
			runOnce(()->setIsOuttakingHigh(true)),
			either(sequence(
				setWristPosRadiansC(wristMaximum).withTimeout(1.5),
				setShoulderPosRadiansC(Math.toRadians(-65))
			), 
			none(), 
			()->getShoulderPosRadians()<Math.toRadians(kShoulderMinWristDownDeg)),
			setShoulderWristExtC(Math.toRadians(16.8), Math.toRadians(28.5), true)
		);
	}

	public CommandBase scoreMidCubeC(){
		return sequence(
			runOnce(()->setIsOuttakingCone(false)),
			runOnce(()->setIsOuttakingHigh(false)),
			setExtendedC(false),
			either(sequence(
				setWristPosRadiansC(wristMaximum).withTimeout(1.5),
				setShoulderPosRadiansC(Math.toRadians(-65))
			), 
			none(), 
			()->getShoulderPosRadians()<Math.toRadians(kShoulderMinWristDownDeg)),
			setShoulderWristExtC(Math.toRadians(-24.5), Math.toRadians(30), false)
		);
	}

	public CommandBase scoreHighCubeC(){
		return sequence(
			runOnce(()->setIsOuttakingCone(false)),
			runOnce(()->setIsOuttakingHigh(true)),
			setExtendedC(false),
            either(sequence(
				setWristPosRadiansC(wristMaximum).withTimeout(1.5),
				setShoulderPosRadiansC(Math.toRadians(-65))
			), 
			none(), 
			()->getShoulderPosRadians()<Math.toRadians(kShoulderMinWristDownDeg)),
			setShoulderWristExtC(Math.toRadians(-7), Math.toRadians(32.3), false)
		);
	}

	public CommandBase pickUpDoubleSubC(){
		return sequence(
			setExtendedC(false),
			either(sequence(
				setWristPosRadiansC(wristMaximum).withTimeout(1.5),
				setShoulderPosRadiansC(Math.toRadians(-65))
			), 
			none(), 
			()->getShoulderPosRadians()<Math.toRadians(kShoulderMinWristDownDeg)),
			setShoulderWristExtC(Math.toRadians(4.7), Math.toRadians(-5), false)
		);
	}
	public CommandBase pickUpSingleSubC(){
		return sequence(
			setExtendedC(false),
			either(sequence(
				setWristPosRadiansC(wristMaximum).withTimeout(1.5),
				setShoulderPosRadiansC(Math.toRadians(-65))
			), 
			none(), 
			()->getShoulderPosRadians()<Math.toRadians(kShoulderMinWristDownDeg)),
			setShoulderWristExtC(Math.toRadians(-23), Math.toRadians(28), false)
		);
	}

    public void log() {
        String states = "Arm/States/";
        SmartDashboard.putNumber(states+"Shoulder Degrees", Math.toDegrees(getShoulderPosRadians()));
        SmartDashboard.putNumber(states+"Shoulder Setpoint Degrees", Math.toDegrees(shoulderPid.getSetpoint().position));
		SmartDashboard.putNumber(states+"Shoulder Goal Degrees", Math.toDegrees(shoulderPid.getGoal().position));
		SmartDashboard.putNumber(states+"Wrist Ground Relative Deg", Math.toDegrees(getWristGroundRelPosRadians()));
        SmartDashboard.putNumber(states+"Wrist Degrees", Math.toDegrees(getWristPosRadians()));
        SmartDashboard.putNumber(states+"Wrist Setpoint Degrees", Math.toDegrees(wristPid.getSetpoint().position));
		SmartDashboard.putNumber(states+"Wrist Goal Degrees", Math.toDegrees(wristPid.getGoal().position));
        SmartDashboard.putNumber(states+"Extension", getExtension());
		SmartDashboard.putBoolean(states+"Extended", getExtended());
        String motors = "Arm/Motors/";
		SmartDashboard.putNumber(motors+"Shoulder A Volts", shoulderMotorA.getAppliedOutput());
		SmartDashboard.putNumber(motors+"Shoulder B Volts", shoulderMotorB.getAppliedOutput());
		SmartDashboard.putNumber(motors+"Shoulder A Current", shoulderMotorA.getOutputCurrent());
		SmartDashboard.putNumber(motors+"Shoulder B Current", shoulderMotorB.getOutputCurrent());
		SmartDashboard.putNumber(motors+"Wrist Volts", wristMotor.getAppliedOutput());
		SmartDashboard.putNumber(motors+"Wrist Current", wristMotor.getOutputCurrent());
    }

    @Override
	public void simulationPeriodic() {
		armSim.simulationPeriodic();
	}


    //----- Dashboard configuration
    //--- Safety angles
    @Config(defaultValueNumeric = kShoulderMinimumDeg)
	void configShoulderMinimumDeg(double shoulderMinimumDeg) {
		this.shoulderMinimum = Math.toRadians(shoulderMinimumDeg);
	}
    @Config(defaultValueNumeric = kShoulderMaximumDeg)
	void configShoulderMaximumDeg(double shoulderMaximumDeg) {
		this.shoulderMaximum = Math.toRadians(shoulderMaximumDeg);
	}
    @Config(defaultValueNumeric = kWristMinimumDeg)
	void configWristMinimumDeg(double wristMinimumDeg) {
		this.wristMinimum = Math.toRadians(wristMinimumDeg);
	}
	@Config(defaultValueNumeric = kWristMaximumDeg)
	void configWristMaximumDeg(double wristMaximumDeg) {
		this.wristMaximum = Math.toRadians(wristMaximumDeg);
	}

	@Config(defaultValueNumeric = kShoulderMinWristDownDeg)
	void configShoulderMinWristDownDeg(double shoulderMinWristDownDeg) {
		this.shoulderMinWristDown = Math.toRadians(shoulderMinWristDownDeg);
	}
	@Config(defaultValueNumeric = kShoulderMinExtendedDeg)
	void configShoulderMinExtDeg(double shoulderMinExtDeg) {
		this.shoulderMinExtended = Math.toRadians(shoulderMinExtDeg);
	}
    @Config(defaultValueNumeric = kShoulderMinExtendedWristDownDeg)
	void configShoulderMinExtWristDownDeg(double shoulderMinExtWristDownDeg) {
		this.shoulderMinExtendedWristDown = Math.toRadians(shoulderMinExtWristDownDeg);
	}

    //--- Feedforward
	private double shoulderks1 = kShoulderks1;
	private double shoulderkg1 = kShoulderkg1;
	private double shoulderkv1 = kShoulderkv1;
	private double shoulderka1 = kShoulderka1;
    private double shoulderks2 = kShoulderks2;
	private double shoulderkg2 = kShoulderkg2;
	private double shoulderkv2 = kShoulderkv2;
	private double shoulderka2 = kShoulderka2;
	@Config(defaultValueNumeric = kShoulderks1)
	void configShoulderks(double ks) {
        if(getExtended()) shoulderks2 = ks;
        else shoulderks1 = ks;
		kShoulderFF = new ArmLerpFeedforward(
            shoulderks1, shoulderkg1, shoulderkv1, shoulderka1,
            shoulderks2, shoulderkg2, shoulderkv2, shoulderka2
        );
	}
	@Config(defaultValueNumeric = kShoulderkg1)
	void configShoulderkg(double kg) {
		if(getExtended()) shoulderkg2 = kg;
        else shoulderkg1 = kg;
		kShoulderFF = new ArmLerpFeedforward(
            shoulderks1, shoulderkg1, shoulderkv1, shoulderka1,
            shoulderks2, shoulderkg2, shoulderkv2, shoulderka2
        );
	}
	@Config(defaultValueNumeric = kShoulderkv1)
	void configShoulderkv(double kv) {
		if(getExtended()) shoulderkv2 = kv;
        else shoulderkv1 = kv;
		kShoulderFF = new ArmLerpFeedforward(
            shoulderks1, shoulderkg1, shoulderkv1, shoulderka1,
            shoulderks2, shoulderkg2, shoulderkv2, shoulderka2
        );
	}
	@Config(defaultValueNumeric = kShoulderka1)
	void configShoulderka(double ka) {
		if(getExtended()) shoulderka2 = ka;
        else shoulderka1 = ka;
		kShoulderFF = new ArmLerpFeedforward(
            shoulderks1, shoulderkg1, shoulderkv1, shoulderka1,
            shoulderks2, shoulderkg2, shoulderkv2, shoulderka2
        );
	}

	private double wristks = kWristks;
	private double wristkg = kWristkg;
	private double wristkv = kWristkv;
	private double wristka = kWristka;
	@Config(defaultValueNumeric = kWristks)
	void configWristks(double ks) {
		wristks = ks;
		kWristFF = new ArmFeedforward(wristks, wristkg, wristkv, wristka);
	}
	@Config(defaultValueNumeric = kWristkg)
	void configWristkg(double kg) {
		wristkg = kg;
		kWristFF = new ArmFeedforward(wristks, wristkg, wristkv, wristka);
	}
	@Config(defaultValueNumeric = kWristkv)
	void configWristkv(double kv) {
		wristkv = kv;
		kWristFF = new ArmFeedforward(wristks, wristkg, wristkv, wristka);
	}
	@Config(defaultValueNumeric = kWristka)
	void configWristka(double ka) {
		wristka = ka;
		kWristFF = new ArmFeedforward(wristks, wristkg, wristkv, wristka);
	}
    //--- Profile constraints
	private double shoulderConfigPIDVelocityConstraint = kShoulderVelocityDeg;
	private double shoulderConfigPIDAccelerationConstraint = kShoulderAccelDeg;
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

	private double wristConfigPIDVelocityConstraint = kWristVelocityDeg;
	private double wristConfigPIDAccelerationConstraint = kWristAccelDeg;
	@Config(defaultValueNumeric = kWristVelocityDeg)
	void configWristPIDVelocityConstraint(double velocity) {
		wristConfigPIDVelocityConstraint = velocity;
		wristPid.setConstraints(new Constraints(Math.toRadians(wristConfigPIDVelocityConstraint), Math.toRadians(wristConfigPIDAccelerationConstraint)));
	}
	@Config(defaultValueNumeric = kWristAccelDeg)
	void configWristPIDAccelerationConstraint(double acceleration) {
		wristConfigPIDAccelerationConstraint = acceleration;
		wristPid.setConstraints(new Constraints(Math.toRadians(wristConfigPIDVelocityConstraint), Math.toRadians(wristConfigPIDAccelerationConstraint)));
	}
	@Config
	void configShoulderTestVolts(double shoulderTestVolts) {
		this.shoulderTestVolts = shoulderTestVolts;
	}
	@Config
	void configWristTestVolts(double wristTestVolts) {
		this.wristTestVolts = wristTestVolts;
	}
}

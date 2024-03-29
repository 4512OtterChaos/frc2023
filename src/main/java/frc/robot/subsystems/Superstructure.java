// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.OCSwerveFollower;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCXboxController;

public class Superstructure {

    private final Arm arm;
    private final SwerveDrive drive;
    private final Intake intake;

    public static final double kSpeedSlow = .45;
    public static final double kSpeedDefault = .7;
    public static final double kSpeedFast = 1.0;
    public static final double kSpeedMax = 1.0;
    
    private double drivespeed = kSpeedDefault;
    private double turnSpeed = kTurnSpeed;
    public static final double kTurnSpeedSlow = 0.15;
    public static final double kTurnSpeed = 0.35;

    public static final double kConeOutakeAngleDecrease = Units.degreesToRadians(16);


    public Superstructure(Arm arm, SwerveDrive drive, Intake intake) {
        this.arm = arm;
        this.drive = drive;
        this.intake = intake;
    }

    public double getDriveSpeed() {
        if (Math.toDegrees(arm.getShoulderPosRadians())>-55){
            return kSpeedSlow;
        }
        return drivespeed;
    }

    public void setDriveSpeed(double targetSpeed) {
        drivespeed = targetSpeed;
    }

    public double getTurnSpeed() {
        if (Math.toDegrees(arm.getShoulderPosRadians())>-55){
            return kTurnSpeedSlow;
        }
        return turnSpeed;
    }

    public void setTurnSpeed(double targetSpeed) {
        turnSpeed = targetSpeed;
    }

    public CommandBase p1() {
        return new OCSwerveFollower(
                drive,
                () -> PathPlanner.generatePath(AutoConstants.kMediumSpeedConfig,
                        PathPoint.fromCurrentHolonomicState(drive.getPose(), drive.getChassisSpeeds()),
                        FieldUtil.allianceMirror(FieldUtil.kGrid1)),
                false);
    }

    public CommandBase cubeConeOuttake() {
        return either(
            sequence(
                runOnce(()->arm.setShoulderPosRadians(arm.shoulderPid.getSetpoint().position-kConeOutakeAngleDecrease)),
                waitSeconds(1),
                intake.setVoltageOutC()
            ), 
            intake.setVoltageOutC(), 
            ()->(arm.getIsOuttakingCone() && arm.getIsOuttakingHigh()));
    }

    public Command rumbleIntakeStall(OCXboxController... controllers) {
        return new FunctionalCommand(
            ()->{},
            ()->{
                if(intake.stallDetection()) {
                    double time = Timer.getFPGATimestamp() / 0.1; // time / x seconds per pulse
                    boolean pulse = ((int) time) % 2 == 0;
                    for(OCXboxController controller : controllers) {
                        if(pulse) controller.rumble(false, 1);
                        else controller.rumble(false, 0);
                    }
                }
                else {
                    for(OCXboxController controller : controllers) {
                        controller.rumble(false, 0);
                    }
                }
            },
            (interrupted)->{
                for(OCXboxController controller : controllers) {
                    controller.rumble(false, 0);
                } 
            },
            ()->false
        );
    }

}

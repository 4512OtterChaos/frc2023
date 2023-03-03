// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveDrive;

public class AutoBalance extends CommandBase {

    private final SwerveDrive drive;

    private static final double kAngleToleranceRads = Math.toRadians(15);

    // private static final double kBalanceSpeed = 0.3;

    private static final double kP = 0.01;
    private static final double kD = 0.01;
    // (we manually use kP because the angle tolerance makes it nonlinear)
    private final PIDController pidPitch = new PIDController(0, 0, kD);
    private final PIDController pidRoll = new PIDController(0, 0, kD);

    public AutoBalance(SwerveDrive drive) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drive = drive;
        addRequirements(drive);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.setIsFieldRelative(true);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pitch = drive.getGyroPitch().getRadians();
        double roll = drive.getGyroRoll().getRadians();
        double vx = 0;
        double vy = 0;
        double omegaRadians = 0;
        // if(Math.abs(pitch) > kAngleToleranceRads){
        //     vx = kBalanceSpeed * -Math.signum(pitch);
        // }
        // if(Math.abs(roll) > kAngleToleranceRads){
        //     vy = kBalanceSpeed * -Math.signum(roll);
        // }
        // if((vx != 0 || vy != 0) && (Math.abs(yaw) > kAngleToleranceDeg)){
        //     omegaRadians = kBalanceSpeed * -Math.signum(yaw);
        // }

        // Calculate slope angle
        if(Math.atan(Math.sqrt(Math.tan(pitch)*Math.tan(pitch) + Math.tan(roll)*Math.tan(roll))) > kAngleToleranceRads) {
            // if outside tolerance, use kP
            vx += kP * pitch;
            vy += kP * roll;
            
        }
        vx += pidPitch.calculate(pitch);
        vy += pidRoll.calculate(roll);

        drive.drive(vx, vy, omegaRadians, true);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
        
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

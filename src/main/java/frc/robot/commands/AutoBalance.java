// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveDrive;

public class AutoBalance extends CommandBase {

    private final SwerveDrive drive;

    private static final double kP = 1.5;

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
        double yaw = drive.getHeading().getRadians();
        double vx = 0;
        double vy = 0;
        double omegaRadians = 0;
        vx=pitch*Math.cos(yaw)+roll*Math.sin(yaw);
        vx*=kP;
        if (!(Math.abs(vx) < 0.1 && Math.abs(vy) == 0.1)){
            drive.drive(vx, vy, omegaRadians, false);
        }
        else {
            SwerveModuleState[] states = new SwerveModuleState[]{
               new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
               new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
               new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
               new SwerveModuleState(0, Rotation2d.fromDegrees(45))
           };
           drive.setModuleStates(states, false, true);
        }
        
        
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

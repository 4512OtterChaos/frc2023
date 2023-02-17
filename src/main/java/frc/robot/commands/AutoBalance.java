// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveDrive;

public class AutoBalance extends CommandBase {
    /** Creates a new AutoBalance. */
    private final SwerveDrive drive;
    private static final double kAngleToleranceDeg = 10;
    private static final double kBalanceSpeed = 0.3;
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
        double pitch = drive.getGyroPitch().getDegrees();
        double roll = drive.getGyroRoll().getDegrees();
        double vx = 0;
        double vy = 0;
        if(Math.abs(pitch) > kAngleToleranceDeg){
            vx = kBalanceSpeed * -Math.signum(pitch);
        }
        if(Math.abs(roll) > kAngleToleranceDeg){
            vy = kBalanceSpeed * -Math.signum(roll);
        }

        drive.drive(vx, vy, 0, true);
        
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }
        
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

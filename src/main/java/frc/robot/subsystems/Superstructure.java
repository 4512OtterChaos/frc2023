// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.robot.auto.AutoConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;

public class Superstructure {

    private final Arm arm;
    private final SwerveDrive drive;
    private final Intake intake;

    public Superstructure(Arm arm, SwerveDrive drive, Intake intake){
        this.arm = arm;
        this.drive = drive;
        this.intake = intake;
    }


    public void periodic() {

    }
    
    public CommandBase p1() {
        // PathPlanner.generatePath(AutoConstants.kMediumSpeedConfig, new PathPoint(drive.getPose(), heading, drive.getHeading(), velocityOverride) point2, points);
        return none();
    }
    

}

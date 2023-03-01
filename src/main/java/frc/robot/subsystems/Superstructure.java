// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.OCSwerveFollower;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;

public class Superstructure {

    private final Arm arm;
    private final SwerveDrive drive;
    private final Intake intake;

    private PathPoint currentPoint;
    private PathPoint endPoint;

    public Superstructure(Arm arm, SwerveDrive drive, Intake intake){
        this.arm = arm;
        this.drive = drive;
        this.intake = intake;

        currentPoint = new PathPoint(drive.getPose().getTranslation(), new Rotation2d(), drive.getPose().getRotation());
        endPoint = new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d());
    }


    public void periodic() {
        currentPoint = new PathPoint(drive.getPose().getTranslation(), new Rotation2d(), drive.getPose().getRotation());
        drive.logTrajectory(
            PathPlanner.generatePath(
                AutoConstants.kMediumSpeedConfig,
                currentPoint,
                new PathPoint(new Translation2d(8, 4), new Rotation2d())
            )
        );
    }
    
    public CommandBase p1() {
        return runOnce(()->{
            var path = (PathPlannerTrajectory)drive.getLogTrajectory();
            new OCSwerveFollower(drive, path, AutoConstants.kMediumSpeedConfig, false).schedule();;
        });
    }
    

}

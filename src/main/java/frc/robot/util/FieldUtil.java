// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands.*;

/** Add your docs here. */
public class FieldUtil {

    public static final PathPoint kTopGridTop = new PathPoint(new Translation2d(2.01, 4.67), Rotation2d.fromDegrees(169.26), Rotation2d.fromDegrees(169.26));
    public static final PathPoint kTopGridMid = new PathPoint(new Translation2d(2.07, 4.41), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));
    public static final PathPoint kTopGridBottom = new PathPoint(new Translation2d(2.07, 3.86), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));
    public static final PathPoint kMiddleGridTop = new PathPoint(new Translation2d(2.07, 3.29), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));
    public static final PathPoint kMiddleGridMid = new PathPoint(new Translation2d(2.07, 2.72), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));
    public static final PathPoint kMiddleGridBottom = new PathPoint(new Translation2d(2.07, 2.22), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));
    public static final PathPoint kBottomGridTop = new PathPoint(new Translation2d(2.07, 1.61), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));
    public static final PathPoint kBottomGridMid = new PathPoint(new Translation2d(2.07, 1.06), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180));
    public static final PathPoint kBottomGridBottom = new PathPoint(new Translation2d(2.13, 0.94), Rotation2d.fromDegrees(-167.14), Rotation2d.fromDegrees(-167.14));
    public static final PathPoint kDoubleSubTop = new PathPoint(new Translation2d(15.38, 7.33), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));
    public static final PathPoint kDoubleSubBottom = new PathPoint(new Translation2d(15.40, 6.29), Rotation2d.fromDegrees(-13.50), Rotation2d.fromDegrees(-13.50));
    public static final PathPoint kSingleSub = new PathPoint(new Translation2d(14.05, 7.00), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90));
    
    public CommandBase generatePath(){
        return none();

    }
}

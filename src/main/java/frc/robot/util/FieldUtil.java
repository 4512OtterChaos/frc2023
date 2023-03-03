// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Helpful utilities related to field dimensions.
 */
public class FieldUtil {

    public static final double kFieldWidth = 8.02;

    public static final double kRobotWidth = Units.inchesToMeters(25 + 3.25*2);
    public static final double kRobotLength = Units.inchesToMeters(28 + 3.25*2);
    public static final double kRobotBumperHeight = Units.inchesToMeters(7);

    // ----- Static field poses
    public static final double kGridAlignX = 1.85;
    public static final double kGrid1AlignY = Units.inchesToMeters(20.19);
    public static final double kGridSpacingY = Units.inchesToMeters(22);
    // Grids 1 - 9, where grid 1 is on the cable protector side of the charging
    // station (right on blue, left on red).
    // To control the curvature approaching the waypoint, use
    // .withPrevControlLength(double)
    public static final PathPoint kGrid1 = new PathPoint(
            new Translation2d(kGridAlignX, kGrid1AlignY), new Rotation2d(Math.PI), new Rotation2d(Math.PI));
    public static final PathPoint kGrid2 = new PathPoint(
            new Translation2d(kGridAlignX, kGrid1AlignY + kGridSpacingY),
            new Rotation2d(Math.PI), new Rotation2d(Math.PI));
    public static final PathPoint kGrid3 = new PathPoint(
            new Translation2d(kGridAlignX, kGrid1AlignY + kGridSpacingY * 2),
            new Rotation2d(Math.PI), new Rotation2d(Math.PI));
    public static final PathPoint kGrid4 = new PathPoint(
            new Translation2d(kGridAlignX, kGrid1AlignY + kGridSpacingY * 3),
            new Rotation2d(Math.PI), new Rotation2d(Math.PI));
    public static final PathPoint kGrid5 = new PathPoint(
            new Translation2d(kGridAlignX, kGrid1AlignY + kGridSpacingY * 4),
            new Rotation2d(Math.PI), new Rotation2d(Math.PI));
    public static final PathPoint kGrid6 = new PathPoint(
            new Translation2d(kGridAlignX, kGrid1AlignY + kGridSpacingY * 5),
            new Rotation2d(Math.PI), new Rotation2d(Math.PI));
    public static final PathPoint kGrid7 = new PathPoint(
            new Translation2d(kGridAlignX, kGrid1AlignY + kGridSpacingY * 6),
            new Rotation2d(Math.PI), new Rotation2d(Math.PI));
    public static final PathPoint kGrid8 = new PathPoint(
            new Translation2d(kGridAlignX, kGrid1AlignY + kGridSpacingY * 7),
            new Rotation2d(Math.PI), new Rotation2d(Math.PI));
    public static final PathPoint kGrid9 = new PathPoint(
            new Translation2d(kGridAlignX, kGrid1AlignY + kGridSpacingY * 8),
            new Rotation2d(Math.PI), new Rotation2d(Math.PI));

    // Double substation shelves
    public static final double kDoubleSubAlignX = 15.3;
    public static final PathPoint kDoubleSubClose = new PathPoint(
            new Translation2d(kDoubleSubAlignX, 6),
            new Rotation2d(), new Rotation2d());
    public static final PathPoint kDoubleSubFar = new PathPoint(
            new Translation2d(kDoubleSubAlignX, 7.3),
            new Rotation2d(), new Rotation2d());
    // Single substation ramp
    public static final PathPoint kSingleSub = new PathPoint(
            new Translation2d(14.2, 6.75),
            Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90));

    // ----- Util methods
    /**
     * Mirrors this PathPoint across the x-axis if on the red alliance.
     */
    public static PathPoint allianceMirror(PathPoint point) {
        // avert your eyes
        try {
            var trlField = PathPoint.class.getDeclaredField("position");
            trlField.setAccessible(true);
            var trl = (Translation2d) trlField.get(point);
            var headingField = PathPoint.class.getDeclaredField("heading");
            headingField.setAccessible(true);
            var heading = (Rotation2d) headingField.get(point);
            var rotField = PathPoint.class.getDeclaredField("holonomicRotation");
            rotField.setAccessible(true);
            var rot = (Rotation2d) rotField.get(point);
            var velField = PathPoint.class.getDeclaredField("velocityOverride");
            velField.setAccessible(true);
            double velOverride = velField.getDouble(point);
            var prevCtrlField = PathPoint.class.getDeclaredField("prevControlLength");
            prevCtrlField.setAccessible(true);
            double prevCtrlLen = prevCtrlField.getDouble(point);
            var nextCtrlField = PathPoint.class.getDeclaredField("nextControlLength");
            nextCtrlField.setAccessible(true);
            double nextCtrlLen = nextCtrlField.getDouble(point);
            if (DriverStation.getAlliance() == Alliance.Red) {
                trl = new Translation2d(trl.getX(), kFieldWidth - trl.getY());
                heading = heading.unaryMinus();
                rot = rot.unaryMinus();
            }
            var pt = new PathPoint(trl, heading, rot, velOverride);
            if (prevCtrlLen > 0) {
                pt = pt.withPrevControlLength(prevCtrlLen);
            }
            if (nextCtrlLen > 0) {
                pt = pt.withNextControlLength(nextCtrlLen);
            }
            return pt;
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("e");
        }
    }

    /**
     * Mirrors this Pose2d across the x-axis if on the red alliance.
     */
    public static Pose2d allianceMirror(Pose2d pose) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            pose = new Pose2d(
                    new Translation2d(pose.getX(), kFieldWidth - pose.getY()),
                    pose.getRotation().unaryMinus());
        }
        return pose;
    }

    /**
     * Returns a list of PathPoints including the current robot point and the target
     * grid point, optionally including intermediate PathPoints to avoid the
     * charging station
     * 
     * @param robotPoint The current robot as a PathPoint
     *                   (see
     *                   {@link PathPoint#fromCurrentHolonomicState(Pose2d, edu.wpi.first.math.kinematics.ChassisSpeeds)})
     * @param gridPoint  The target grid alignment as a PathPoint
     */
    public static List<PathPoint> avoidChargingStation(PathPoint robotPoint, PathPoint gridPoint) {
        List<PathPoint> path = new ArrayList<>();
        try{
            boolean opposingLoadingStation = false;
            boolean ourLoadingStation = false;
            boolean chargingStationTop = false;
            boolean chargingStationBottom = false;
            var trlField = PathPoint.class.getDeclaredField("position");
            trlField.setAccessible(true);
            var trl = (Translation2d) trlField.get(robotPoint);
            var targetTrl = (Translation2d) trlField.get(gridPoint);
            double robotX = trl.getX();
            double robotY = trl.getY();
            if (robotY > 5.5 && robotX < 4.25){
                opposingLoadingStation = true;
                // .add(robotPoint);
                // .add(new PathPoint(
                //         new Translation2d(4.64, 5.97), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
                // .add(new PathPoint(
                //         new Translation2d(4, 4.65), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
                // .add(new PathPoint(
                //         new Translation2d(2.3, 4.6), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
                // .add(gridPoint);
            }
            else if (robotY > 5.5 && robotX > 12.7){
                ourLoadingStation = true;
                // .add(robotPoint);
                // .add(new PathPoint(
                //         new Translation2d(10.7, 6.35), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
                // .add(new PathPoint(
                //         new Translation2d(4, 4.65), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
                // .add(new PathPoint(
                //         new Translation2d(2.3, 4.6), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
                // .add(gridPoint);
            }
            else if (robotX > 2.35 && robotY > 2.7){
                chargingStationTop = true;
                // .add(robotPoint);
                // .add(new PathPoint(
                //         new Translation2d(5.55, 4.65), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
                // .add(new PathPoint(
                //         new Translation2d(2.3, 4.6), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
                // .add(gridPoint);
            }
            else if (robotX > 2.35 && robotY > 2.7){
                chargingStationBottom = true;
                // .add(robotPoint);
                // .add(new PathPoint(
                //         new Translation2d(5.55, 0.85), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
                // .add(new PathPoint(
                //         new Translation2d(2.3, 0.75), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
                // .add(gridPoint);
            }
            else {
                // .add(robotPoint);
                // .add(gridPoint);
            }
            return path;
        } catch (Exception e) {
                e.printStackTrace();
                throw new RuntimeException("e");
        }
    }
}

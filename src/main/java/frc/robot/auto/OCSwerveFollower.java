// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.SwerveDrive;

public class OCSwerveFollower extends CommandBase {
    
    private final SwerveDrive drivetrain;
    private PathPlannerTrajectory path;
    private Supplier<PathPlannerTrajectory> pathSupplier;
    private final boolean resetOdom;

    private CommandBase controllerCommand = Commands.none();
    private boolean initialized = false;

    public OCSwerveFollower(
            SwerveDrive drivetrain, String pathName,
            PathConstraints constraints, boolean resetOdom) {
        this.drivetrain = drivetrain;
        path = PathPlanner.loadPath(pathName, constraints);
        pathSupplier = null;
        this.resetOdom = resetOdom;

        addRequirements(drivetrain);
    }
    public OCSwerveFollower(
            SwerveDrive drivetrain, Supplier<PathPlannerTrajectory> pathSupplier,
            boolean resetOdom) {
        this.drivetrain = drivetrain;
        this.path = null;
        this.pathSupplier = pathSupplier;
        this.resetOdom = resetOdom;

        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        if(pathSupplier != null) {
            path = pathSupplier.get();
        }
        if(path == null) {
            end(false);
            DriverStation.reportError("Attempted to follow null path!", true);
            return;
        }
        if(pathSupplier == null && !initialized) {
            path = PathPlannerTrajectory.transformTrajectoryForAlliance(
                path,
                DriverStation.getAlliance());
        }

        if(resetOdom) drivetrain.resetOdometry(path.getInitialHolonomicPose());
        drivetrain.logTrajectory(path);

        controllerCommand = new PPSwerveControllerCommand(
            path,
            drivetrain::getPose,
            drivetrain.getXController(),
            drivetrain.getYController(),
            drivetrain.getRotController(),
            (chassisSpeeds)->drivetrain.setChassisSpeeds(chassisSpeeds, false, true),
            false,
            drivetrain
        );
        controllerCommand.initialize();
        initialized = true;
    }
    
    @Override
    public void execute() {
        controllerCommand.execute();
    }
    
    @Override
    public void end(boolean interrupted) {
        controllerCommand.end(interrupted);
        drivetrain.logTrajectory(null);
    }
    
    @Override
    public boolean isFinished() {
        return controllerCommand.isFinished();
    }
}

package frc.robot.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;

public class AutoOptions {
    
    // list of choosable commands that decides what is run in auto
    private SendableChooser<CommandBase> autoOptions = new SendableChooser<>();

    public AutoOptions(SwerveDrive drive, Intake intake){

        autoOptions.setDefaultOption("Omegalul",
            new InstantCommand(()->drive.stop(), drive)
        );
        autoOptions.addOption("ZTornado",
            new OCSwerveFollower(
                    drive, 
                    "ZTornado", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ) 
        );
        autoOptions.addOption("NolanExample",
            sequence(
                new OCSwerveFollower(
                    drive, 
                    "NolanExample1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ),
                intake.setSpeedC(0.75).withTimeout(2),
                new OCSwerveFollower(
                    drive, 
                    "NolanExample2", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                )
            )
        );

    }
    
    public CommandBase getAutoCommand(){
        var cmd = autoOptions.getSelected();
        if(cmd == null){
            cmd = none();
        }
        return cmd;
    }


    public void submit(){
        SmartDashboard.putData("Auto Options", autoOptions);
    }
}

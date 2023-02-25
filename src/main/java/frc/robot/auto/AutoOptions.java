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
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;

public class AutoOptions {
    
    // list of choosable commands that decides what is run in auto
    private SendableChooser<CommandBase> autoOptions = new SendableChooser<>();

    public AutoOptions(SwerveDrive drive, Intake intake, Arm arm){

        autoOptions.setDefaultOption("Nothing",
            new InstantCommand(()->drive.stop(), drive)
        );

        autoOptions.addOption("NolanExample",
            sequence(
                new OCSwerveFollower(
                    drive, 
                    "NolanExample1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ),
                intake.setVoltageC(0.75).withTimeout(2),
                new OCSwerveFollower(
                    drive, 
                    "NolanExample2", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                )
            )
        );
        autoOptions.addOption("T1Cone1CubeEngaged",
            sequence(
                arm.scoreUpperC().withTimeout(4),
                intake.setVoltageC(-0.25).withTimeout(2),
                arm.inC().withTimeout(4),
                new OCSwerveFollower(
                    drive, 
                    "T1Cone1CubeEngaged1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ),
                parallel(
                    intake.setVoltageC(0.75).withTimeout(5),
                    arm.pickUpGroundC().withTimeout(5)
                ),
                arm.inC().withTimeout(4),
                new OCSwerveFollower(
                    drive, 
                    "T1Cone1CubeEngaged2", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                ),
                arm.scoreUpperC().withTimeout(4),
                intake.setVoltageC(-0.25).withTimeout(2),
                arm.inC().withTimeout(4),
                new OCSwerveFollower(
                    drive, 
                    "T1Cone1CubeEngaged3", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                )
            )
        );
        autoOptions.addOption("1CubeEngaged",
            sequence(
                arm.scoreUpperC().withTimeout(4),
                intake.setVoltageC(-0.25).withTimeout(2),
                arm.inC().withTimeout(4),
                new OCSwerveFollower(
                    drive, 
                    "1CubeEngaged", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                )
            )
        );
        autoOptions.addOption("1CubeTaxiEngaged",
            sequence(
                arm.scoreUpperC().withTimeout(4),
                intake.setVoltageC(-0.25).withTimeout(2),
                arm.inC().withTimeout(4),
                new OCSwerveFollower(
                    drive, 
                    "1CubeTaxiEngaged", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                )
            )
        );
        // autoOptions.addOption("AMOGUS",
        //     sequence(
        //         arm.scoreUpperC().withTimeout(4),
        //         intake.setVoltageC(-0.25).withTimeout(2),
        //         arm.inC().withTimeout(4),
        //         new OCSwerveFollower(
        //             drive, 
        //             "AMOGUS", 
        //             AutoConstants.kMaxLinearSpeed,
        //             true
        //         )
        //     )
        // );
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

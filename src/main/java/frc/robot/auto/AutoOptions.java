package frc.robot.auto;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;

public class AutoOptions {
    
    // list of choosable commands that decides what is run in auto
    private SendableChooser<CommandBase> autoOptions = new SendableChooser<>();

    public AutoOptions(SwerveDrive drive){
        addDriveOnlyOptions(drive);
    }

    private void addDriveOnlyOptions(SwerveDrive drive) {
        autoOptions.setDefaultOption("Nothing",
            new InstantCommand(()->drive.stop(), drive)
        );
        autoOptions.addOption("TaxiSideDumb",
            new OCSwerveFollower(
                drive, 
                "TaxiSideDumb", 
                AutoConstants.kMediumSpeedConfig,
                true
            )
        
        );
        autoOptions.addOption("PushTaxiSideDumb",
            new OCSwerveFollower(
                drive, 
                "PushTaxiSideDumb", 
                AutoConstants.kMediumSpeedConfig,
                true
            )
        );
        autoOptions.addOption("Engaged",
            sequence(
                new OCSwerveFollower(
                    drive, 
                    "Engaged", 
                    AutoConstants.kSlowSpeedConfig,
                    true
                ),
                new AutoBalance(drive)                   
            )
        );
        autoOptions.addOption("TaxiEngaged",
            sequence(
                new OCSwerveFollower(
                    drive, 
                    "TaxiEngaged1", 
                    AutoConstants.kSlowSpeedConfig,
                    true
                ),
                waitSeconds(0.7),
                new OCSwerveFollower(
                    drive, 
                    "TaxiEngaged2", 
                    AutoConstants.kSlowSpeedConfig,
                    true
                ),
                new AutoBalance(drive)                   
            )
        );
        autoOptions.addOption("PushEngaged",
            sequence(
                new OCSwerveFollower(
                    drive, 
                    "PushEngaged", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ),
                new AutoBalance(drive)
            )
        );
        autoOptions.addOption("PushTaxiEngaged",
            sequence(
                new OCSwerveFollower(
                    drive, 
                    "PushTaxiEngaged", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ),
                new AutoBalance(drive)
            )
        );
    }

    public AutoOptions(SwerveDrive drive, Intake intake, Arm arm){
        addDriveOnlyOptions(drive);
                
        autoOptions.addOption("NoBumpCubetaxi",
            sequence(
                runOnce(()->intake.setVoltage(1.93), intake),
                arm.scoreHighCubeC().withTimeout(4),
                waitSeconds(0.5),
                intake.setVoltageOutC().withTimeout(2),
                arm.inC().withTimeout(4),
                new OCSwerveFollower(
                    drive, 
                    "T1CubeTaxi", 
                    AutoConstants.kSlowSpeedConfig,
                    true
                )
            )
        );
        autoOptions.addOption("BumpCubetaxi",
            sequence(
                runOnce(()->intake.setVoltage(1.93), intake),
                arm.scoreHighCubeC().withTimeout(3),
                waitSeconds(0.5),
                intake.setVoltageOutC().withTimeout(1.5),
                arm.inC().withTimeout(3),
                new OCSwerveFollower(
                    drive, 
                    "B1CubeTaxi", 
                    AutoConstants.kSlowSpeedConfig,
                    true
                )
            )
        );
        autoOptions.addOption("MiddleCubeEngaged",
            sequence(
                runOnce(()->intake.setVoltage(1.93), intake),
                arm.scoreHighCubeC().withTimeout(4),
                waitSeconds(0.5),
                intake.setVoltageC(-0.25).withTimeout(2),
                arm.inC().withTimeout(4),
                new OCSwerveFollower(
                    drive, 
                    "M1CubeEngaged", 
                    AutoConstants.kSlowSpeedConfig,
                    true
                ),
                new AutoBalance(drive)
            )
        );

        

        autoOptions.addOption("BumpDoubleCube",
            sequence(
                runOnce(()->intake.setVoltage(1.93), intake),
                arm.scoreHighCubeC().withTimeout(3),
                waitSeconds(0.25),
                intake.setVoltageOutC().withTimeout(1.5),
                arm.inC().withTimeout(3),
                parallel(
                    sequence(
                        waitSeconds(1.5),
                        arm.pickUpCubeGroundC()
                    ),
                    new OCSwerveFollower(
                        drive, 
                        "B1CubeTaxi", 
                        AutoConstants.kSlowSpeedConfig,
                        true
                    )
                ),
                parallel(
                    sequence(
                        intake.setVoltageInC().withTimeout(3),
                        runOnce(()->intake.setVoltage(1.93), intake),
                        arm.coneInC().withTimeout(1)
                    ),
                    new OCSwerveFollower(
                        drive, 
                        "B1CubeTaxi2", 
                        AutoConstants.kSlowSpeedConfig,
                        false
                    )
                ),
                intake.setVoltageOutC().withTimeout(1),
                intake.setVoltageC(0),
                arm.inC()
            )
        );
        // autoOptions.addOption("MiddleCubetaxi",
        //     sequence(
        //         arm.scoreHighCubeC().withTimeout(4),
        //         intake.setVoltageOutC().withTimeout(2),
        //         arm.inC().withTimeout(4),
        //         new OCSwerveFollower(
        //             drive, 
        //             "M1Cubetaxi", 
        //             AutoConstants.kMediumSpeedConfig,
        //             true
        //         )
        //     )
        // );
        // autoOptions.addOption("NoBump1Cone1Cube",
        //     sequence(
        //         arm.scoreUpperConeC().withTimeout(3),
        //         intake.setVoltageC(-0.25).withTimeout(1),
        //         arm.inC().withTimeout(2),
        //         new OCSwerveFollower(
        //             drive, 
        //             "T1Cone1CubeEngaged1", 
        //             AutoConstants.kMediumSpeedConfig,
        //             true
        //         ),
        //         parallel(
        //             intake.setVoltageC(0.75).withTimeout(5),
        //             arm.pickUpGroundC().withTimeout(5)
        //         ),
        //         arm.inC().withTimeout(4),
        //         new OCSwerveFollower(
        //             drive, 
        //             "T1Cone1CubeEngaged2", 
        //             AutoConstants.kMediumSpeedConfig,
        //             false
        //         ),
        //         arm.scoreUpperConeC().withTimeout(4),
        //         intake.setVoltageC(-0.25).withTimeout(2),
        //         arm.inC().withTimeout(4),
        //         new OCSwerveFollower(
        //             drive, 
        //             "T1Cone1CubeEngaged3", 
        //             AutoConstants.kMediumSpeedConfig,
        //             false
        //         )
        //     )
        // );
        // autoOptions.addOption("Middle1CubeTaxiEngaged",
        //     sequence(
        //         arm.scoreUpperConeC().withTimeout(4),
        //         intake.setVoltageC(-0.25).withTimeout(2),
        //         arm.inC().withTimeout(4),
        //         new OCSwerveFollower(
        //             drive, 
        //             "M1CubeTaxiEngaged", 
        //             AutoConstants.kMediumSpeedConfig,
        //             true
        //         )
        //     )
        // );
        // autoOptions.addOption("AMOGUS",
        //     sequence(
        //         arm.scoreUpperConeC().withTimeout(4),
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
        // autoOptions.addOption("NoBumptaxi",
        //     sequence(

        //         new OCSwerveFollower(
        //             drive, 
        //             "Ttaxi", 
        //             AutoConstants.kMediumSpeedConfig,
        //             true
        //         )
        //     )
        // );
        // autoOptions.addOption("Middletaxi",
        //     sequence(

        //         new OCSwerveFollower(
        //             drive, 
        //             "Mtaxi", 
        //             AutoConstants.kMediumSpeedConfig,
        //             true
        //         )
        //     )
        // );
        // autoOptions.addOption("Bumptaxi",
        //     sequence(

        //         new OCSwerveFollower(
        //             drive, 
        //             "Btaxi", 
        //             AutoConstants.kMediumSpeedConfig,
        //             true
        //         )@
        //     )
        // );
        // autoOptions.addOption("NoBump1Conetaxi",
        //     sequence(
        //         arm.scoreUpperConeC().withTimeout(4),
        //         intake.setVoltageC(-0.25).withTimeout(2),
        //         arm.inC().withTimeout(4),
        //         new OCSwerveFollower(
        //             drive, 
        //             "T1Conetaxi", 
        //             AutoConstants.kMediumSpeedConfig,
        //             true
        //         )
        //     )
        // );
        // autoOptions.addOption("Middle1Conetaxi",
        //     sequence(
        //         arm.scoreUpperConeC().withTimeout(4),
        //         intake.setVoltageC(-0.25).withTimeout(2),
        //         arm.inC().withTimeout(4),
        //         new OCSwerveFollower(
        //             drive, 
        //             "M1Conetaxi", 
        //             AutoConstants.kMediumSpeedConfig,
        //             true
        //         )
        //     )
        // );
        // autoOptions.addOption("Bump1Conetaxi",
        //     sequence(
        //         arm.scoreUpperConeC().withTimeout(4),
        //         intake.setVoltageC(-0.25).withTimeout(2),
        //         arm.inC().withTimeout(4),
        //         new OCSwerveFollower(
        //             drive, 
        //             "B1Conetaxi", 
        //             AutoConstants.kMediumSpeedConfig,
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

package frc.robot;

import frc.robot.auto.AutoOptions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LogUtil;
import frc.robot.util.OCXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotContainer {
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final SwerveDrive drive = new SwerveDrive(); 
    private final OCXboxController driver = new OCXboxController(0);
    private final Field2d field = new Field2d();
    private final AutoOptions autoOptions = new AutoOptions(drive, intake);

    
    public RobotContainer() {
        configureEventBinds();
        configureDriverBinds(driver);
        SmartDashboard.putData("field", field);

        autoOptions.submit();
    }
    
    private void configureEventBinds() {
        
    }

    private void configureDriverBinds(OCXboxController controller) {

        // toggle between field-relative and robot-relative control
        controller.back().onTrue(runOnce(()->{
            drive.setIsFieldRelative(!drive.getIsFieldRelative());
        }));

        // reset the robot heading to 0
        controller.start().onTrue(runOnce(()->{
            drive.resetOdometry(
                new Pose2d(
                    drive.getPose().getTranslation(),
                    new Rotation2d()
                )
            );
        }));

        drive.setDefaultCommand(
            run(()->
                drive.drive(
                    controller.getForward()*drive.getMaxLinearVelocityMeters(),
                    controller.getStrafe()*drive.getMaxLinearVelocityMeters(),
                    controller.getTurn()*drive.getMaxAngularVelocityRadians(),
                    false
                ),
                drive
            )
        );
        // controller.rightBumper()
        //     .whileTrue(run(()->arm.setSpeed(0.5), arm));
        // controller.leftBumper()
        //     .whileTrue(run(()->arm.setSpeed(-0.5), arm));

        controller.a()
            .whileTrue(arm.setShoulderPosRadiansC(Units.degreesToRadians(-25)));
        controller.b()
            .onTrue(arm.toggleExstensionExtendedC());
        controller.x()
            .whileTrue(arm.setShoulderPosRadiansC(Units.degreesToRadians(0)));
        controller.y()
            .whileTrue(arm.setShoulderPosRadiansC(Units.degreesToRadians(25)));

        controller.povUp()
            .whileTrue(arm.setWristPosRadiansC(Units.degreesToRadians(25)));
        controller.povLeft()
            .whileTrue(arm.setWristPosRadiansC(Units.degreesToRadians(0)));
        controller.povDown()
            .whileTrue(arm.setWristPosRadiansC(Units.degreesToRadians(-25)));
        controller.povRight()
            .whileTrue(arm.setArmStateC(Math.toRadians(-60), Math.toRadians(-45), arm.getExtensionState()));


    }

    public void periodic() {
        field.setRobotPose(drive.getPose());
        LogUtil.logPose("drivePose2d", drive.getPose());
    }
    public CommandBase getAutoCommand(){
        return autoOptions.getAutoCommand();
    }
    
}

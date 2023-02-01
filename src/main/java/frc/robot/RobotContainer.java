package frc.robot;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.OCXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final Arm arm = new Arm();
    private final SwerveDrive drive = new SwerveDrive(); 
    private final OCXboxController driver = new OCXboxController(0);
    private final Field2d field = new Field2d();
    
    public RobotContainer() {
        configureEventBinds();
        configureDriverBinds(driver);
        SmartDashboard.putData("field", field);
    }
    
    private void configureEventBinds() {
        
    }

    private void configureDriverBinds(OCXboxController controller) {
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
        controller.x()
            .whileTrue(arm.setShoulderPosRadiansC(Units.degreesToRadians(0)));
        controller.y()
            .whileTrue(arm.setShoulderPosRadiansC(Units.degreesToRadians(25)));
        controller.povUp()
            .whileTrue(arm.setWristPosRadiansC(Units.degreesToRadians(-25)));
        controller.povLeft()
            .whileTrue(arm.setWristPosRadiansC(Units.degreesToRadians(0)));
        controller.povDown()
            .whileTrue(arm.setWristPosRadiansC(Units.degreesToRadians(25)));


    }

    public void periodic() {
        field.setRobotPose(drive.getPose());
    }
}

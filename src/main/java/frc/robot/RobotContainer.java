package frc.robot;

import frc.robot.auto.AutoOptions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.intake.Intake;
import frc.robot.util.LogUtil;
import frc.robot.util.OCXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
    private final PhotonCamera camera = new PhotonCamera("camera");
    private final PhotonPoseEstimator photonEstimator;
    private final AprilTagFieldLayout tagLayout;
    
    public RobotContainer() {
        
        try {
            tagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch(Exception e) {
            throw new RuntimeException("AprilTagFieldLayout loading failed!", e);
        }
        photonEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, new Transform3d());
        
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
        controller.rightBumper()
            .whileTrue(run(()->arm.setShoulderPosRadians(arm.shoulderPid.getGoal().position+Units.degreesToRadians(0.5)), arm));
        controller.leftBumper()
            .whileTrue(run(()->arm.setShoulderPosRadians(arm.shoulderPid.getGoal().position-Units.degreesToRadians(0.5)), arm));

        controller.rightTrigger(0.2)
            .whileTrue(run(()->arm.setWristPosRadians(arm.wristPid.getGoal().position+Units.degreesToRadians(0.5)), arm));
        controller.leftTrigger(0.2)
            .whileTrue(run(()->arm.setWristPosRadians(arm.wristPid.getGoal().position-Units.degreesToRadians(0.5)), arm));

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
            .whileTrue(
                sequence(
                    arm.setShoulderPosRadiansC(Units.degreesToRadians(-600)),
                    arm.setWristPosRadiansC(Units.degreesToRadians(-45))
                )
            );


    }

    public void periodic() {
        field.setRobotPose(drive.getPose());
        LogUtil.logPose("drivePose2d", drive.getPose());
        camera.getLatestResult().getTargets();
        photonEstimator.update().ifPresent(est -> {
            drive.addVisionMeasurement(est.estimatedPose.toPose2d(), Timer.getFPGATimestamp() - est.timestampSeconds);
        });
    }
    public CommandBase getAutoCommand(){
        return autoOptions.getAutoCommand();
    }
    
}

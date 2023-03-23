package frc.robot;

import frc.robot.auto.AutoOptions;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.TeleopDriveAngle;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.OCXboxController;
import io.github.oblarg.oblog.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotContainer {
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final SwerveDrive drive = new SwerveDrive(); 
    private final Superstructure superstructure = new Superstructure(arm, drive, intake);
    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    private final OCXboxController driver = new OCXboxController(0);
    private final OCXboxController operator = new OCXboxController(1);
    private final OCXboxController king = new OCXboxController(0);

    private final AutoOptions autoOptions = new AutoOptions(drive, intake, arm);
    // private final AutoOptions autoOptions = new Auto Options(intake, arm);


    private final Field2d field = new Field2d();
    
    // private final PhotonCamera camera = new PhotonCamera("camera");
    private final SimPhotonCamera simCamera = new SimPhotonCamera("camera");
    private final SimVisionSystem visionSim = new SimVisionSystem("camera", 90, new Transform3d(), 10, 640, 480, .01);
    // private final PhotonPoseEstimator photonEstimator;
    private final AprilTagFieldLayout tagLayout;
    
    public RobotContainer() {
        
        try {
            tagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch(Exception e) {
            throw new RuntimeException("AprilTagFieldLayout loading failed!", e);
        }
        // photonEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, camera, new Transform3d());
        visionSim.addVisionTargets(tagLayout);
        
        configureEventBinds();
        if (Robot.isReal()){
            configureDriverBinds(driver);
            configureOperatorBinds(operator);
        }
        else{
            configureDriverBinds(king);
            configureOperatorBinds(king);
        }
        
        SmartDashboard.putData("field", field);

        autoOptions.submit();

        Logger.configureLogging(this);

        // uncomment this line for tuning mode
        Logger.configureConfig(this);
    }

    public void periodic() {
        // superstructure.periodic();
        field.setRobotPose(drive.getPose());
        field.getObject("trajectory").setTrajectory(drive.getLogTrajectory());
        
        // camera.getLatestResult().getTargets();
        // photonEstimator.update().ifPresent(est -> {
        //     drive.addVisionMeasurement(est.estimatedPose.toPose2d(), Timer.getFPGATimestamp() - est.timestampSeconds);
        // });
        
    }
    
    private void configureEventBinds() {

        intake.setDefaultCommand(intake.setVoltageC(0.75));
        
    }

    private void configureDriverBinds(OCXboxController controller) {

        drive.setDefaultCommand(
            run(()->
                drive.drive(
                    controller.getForward()*drive.getMaxLinearVelocityMeters(),
                    controller.getStrafe()*drive.getMaxLinearVelocityMeters(),
                    controller.getTurn()*drive.getMaxAngularVelocityRadians(),
                    true
                ),
                drive
            )
        );

        controller.povUp()
            .onTrue(run(()->
                drive.drive(
                    controller.getForward()*drive.getMaxLinearVelocityMeters(),
                    controller.getStrafe()*drive.getMaxLinearVelocityMeters(),
                    new Rotation2d(),
                    true
                ),
                drive
            )
            .beforeStarting(()->drive.resetPathController())
            .until(()->Math.abs(controller.getRightX())>0.25)
        );
        controller.povDown()
            .onTrue(run(()->
                drive.drive(
                    controller.getForward()*drive.getMaxLinearVelocityMeters(),
                    controller.getStrafe()*drive.getMaxLinearVelocityMeters(),
                    new Rotation2d(Math.PI),
                    true
                ),
                drive
            )
            .beforeStarting(()->drive.resetPathController())
            .until(()->Math.abs(controller.getRightX())>0.25)
        );

        // push-to-change driving "speed"
        controller.rightBumper()
            .onTrue(runOnce(()->controller.setDriveSpeed(OCXboxController.kSpeedFast)))
            .onFalse(runOnce(()->controller.setDriveSpeed(OCXboxController.kSpeedDefault)));

        controller.leftBumper()
            .onTrue(runOnce(()->{
                controller.setDriveSpeed(OCXboxController.kSpeedSlow);
                controller.setTurnSpeed(OCXboxController.kTurnSpeedSlow);
            }))
            .onFalse(runOnce(()->{
                controller.setDriveSpeed(OCXboxController.kSpeedDefault);
                controller.setTurnSpeed(OCXboxController.kTurnSpeed);
            }));


        // controller.back()
        // .onTrue(
        //     superstructure.p1()
        // );

        // reset the robot heading to 0
        controller.start()
            .onTrue(runOnce(()->
                drive.resetOdometry(
                    new Pose2d(
                        drive.getPose().getTranslation(),
                        new Rotation2d()
                    )
                )
            )
        );

        // controller.back()
        //     .onTrue(runOnce(
        //         ()->drive.setIsFieldRelative(!drive.getIsFieldRelative())
        //     ));
        
        // lock the modules in a "X" alignment
        controller.x().whileTrue(run(()->{
           SwerveModuleState[] states = new SwerveModuleState[]{
               new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
               new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
               new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
               new SwerveModuleState(0, Rotation2d.fromDegrees(45))
           };
           drive.setModuleStates(states, false, true);
        }, drive));
        
        controller.y()
            .whileTrue(new AutoBalance(drive));
   
    }

    private void configureOperatorBinds(OCXboxController controller){
        controller.start()
            .whileTrue(run(()->arm.shoulderTestVolts=2,arm))
            .onFalse(runOnce(()->arm.shoulderTestVolts=0,arm));
        controller.back()
            .whileTrue(run(()->arm.shoulderTestVolts=-2,arm))
            .onFalse(runOnce(()->arm.shoulderTestVolts=0,arm));
        controller.rightTrigger()
            .whileTrue(intake.setVoltageInC());
        controller.leftTrigger()
            .whileTrue(intake.setVoltageOutC());

        controller.a()
            .onTrue(arm.pickUpGroundC());
        controller.b()
            .onTrue(arm.pickUpDoubleSubC());
        controller.x()
            .onTrue(arm.scoreMidC());
        controller.y()
            .onTrue(arm.scoreUpperC());
        

        controller.povDown()
            .onTrue(arm.inC());


        controller.leftBumper()
            .whileTrue(run(()->arm.setShoulderPosRadians(arm.shoulderPid.getSetpoint().position-0.05)));
        controller.rightBumper()
            .whileTrue(run(()->arm.setShoulderPosRadians(arm.shoulderPid.getSetpoint().position+0.05)));
            
    }
 
    public CommandBase getAutoCommand(){
        return autoOptions.getAutoCommand();
    }

    public void log() {
        Logger.updateEntries();
        drive.log();

        arm.log();
    }

    public void simulationPeriodic(){
        // visionSim.processFrame(drive.getPose());
    }
}

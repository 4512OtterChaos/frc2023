package frc.robot;

import frc.robot.auto.AutoOptions;
import frc.robot.commands.AutoBalance;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    // private final AutoOptions autoOptions = new AutoOptions(drive);
    private final AutoOptions autoOptions = new AutoOptions(drive, intake, arm);


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
        compressor.enableAnalog(100, 120);
    }

    public void initMode(boolean auto) {
        arm.init(auto);
        
        
    }
    public void periodic() {
        field.setRobotPose(drive.getPose());
        field.getObject("trajectory").setTrajectory(drive.getLogTrajectory());
        
        // camera.getLatestResult().getTargets();
        // photonEstimator.update().ifPresent(est -> {
        //     drive.addVisionMeasurement(est.estimatedPose.toPose2d(), Timer.getFPGATimestamp() - est.timestampSeconds);
        // });
        
    }
    
    private void configureEventBinds() {

        intake.setDefaultCommand(run(()->{
            if(Math.toDegrees(arm.getShoulderPosRadians()) > -80){
                intake.setVoltage(1.93);
            }
            else intake.setVoltage(0);
        }, intake));
        
    }

    private void configureDriverBinds(OCXboxController controller) {

        drive.setDefaultCommand(
            run(()->
                drive.drive(
                    controller.getLeftY(2) * superstructure.getDriveSpeed()*drive.getMaxLinearVelocityMeters(),
                    controller.getLeftX(2) * superstructure.getDriveSpeed()*drive.getMaxLinearVelocityMeters(),
                    controller.getRightX(2) * superstructure.getTurnSpeed()*drive.getMaxAngularVelocityRadians(),
                    true
                ),
                drive
            )
        );

        controller.y()
            .onTrue(run(()->
                drive.drive(
                    controller.getLeftY(2) * superstructure.getDriveSpeed()*drive.getMaxLinearVelocityMeters(),
                    controller.getLeftX(2) * superstructure.getDriveSpeed()*drive.getMaxLinearVelocityMeters(),
                    new Rotation2d(),
                    true
                ),
                drive
            )
            .beforeStarting(()->drive.resetPathController())
            .until(()->Math.abs(controller.getRightX())>0.25)
        );
        controller.a()
            .onTrue(run(()->
                drive.drive(
                    controller.getLeftY(2) * superstructure.getDriveSpeed()*drive.getMaxLinearVelocityMeters(),
                    controller.getLeftX(2) * superstructure.getDriveSpeed()*drive.getMaxLinearVelocityMeters(),
                    new Rotation2d(Math.PI),
                    true
                ),
                drive
            )
            .beforeStarting(()->drive.resetPathController())
            .until(()->Math.abs(controller.getRightX())>0.25)
        );
        controller.b()
            .onTrue(run(()->
                drive.drive(
                    controller.getLeftY(2) * superstructure.getDriveSpeed()*drive.getMaxLinearVelocityMeters(),
                    controller.getLeftX(2) * superstructure.getDriveSpeed()*drive.getMaxLinearVelocityMeters(),
                    new Rotation2d((-Math.PI)/2),
                    true
                ),
                drive
            )
            .beforeStarting(()->drive.resetPathController())
            .until(()->Math.abs(controller.getRightX())>0.25)
        );
        controller.x()
            .onTrue(run(()->
                drive.drive(
                    controller.getLeftY(2) * superstructure.getDriveSpeed()*drive.getMaxLinearVelocityMeters(),
                    controller.getLeftX(2) * superstructure.getDriveSpeed()*drive.getMaxLinearVelocityMeters(),
                    new Rotation2d((Math.PI)/2),
                    true
                ),
                drive
            )
            .beforeStarting(()->drive.resetPathController())
            .until(()->Math.abs(controller.getRightX())>0.25)
        );

        // push-to-change driving "speed"
        controller.rightBumper()
            .onTrue(runOnce(()->superstructure.setDriveSpeed(Superstructure.kSpeedFast)))
            .onFalse(runOnce(()->superstructure.setDriveSpeed(Superstructure.kSpeedDefault)));

        controller.leftBumper()
            .onTrue(runOnce(()->{
                superstructure.setDriveSpeed(Superstructure.kSpeedSlow);
                superstructure.setTurnSpeed(Superstructure.kTurnSpeedSlow);
            }))
            .onFalse(runOnce(()->{
                superstructure.setDriveSpeed(Superstructure.kSpeedDefault);
                superstructure.setTurnSpeed(Superstructure.kTurnSpeed);
            }));

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
        // .onTrue(
            // superstructure.p1()
        // );

        // controller.back()
        //     .onTrue(runOnce(
        //         ()->drive.setIsFieldRelative(!drive.getIsFieldRelative())
        //     ));
        
        // // lock the modules in a "X" alignment
        // controller.x().whileTrue(run(()->{
        //    SwerveModuleState[] states = new SwerveModuleState[]{
        //        new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
        //        new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
        //        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        //        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        //    };
        //    drive.setModuleStates(states, false, true);
        // }, drive));
        
        // controller.b()
        //     .whileTrue(new AutoBalance(drive));
   
    }

    private void configureOperatorBinds(OCXboxController controller){
        controller.rightTrigger()
            .whileTrue(intake.setVoltageInC());
        controller.leftTrigger()
            .whileTrue(superstructure.cubeConeOuttake())
            .onFalse(either(
                either(
                    arm.scoreHighConeC(), 
                    arm.scoreMidConeC(), 
                    ()->arm.getIsOuttakingHigh()), 
                none(), 
                ()->arm.getIsOuttakingCone()));
        controller.a()
            .onTrue(arm.scoreMidCubeC());
        controller.b()
            .onTrue(arm.scoreHighCubeC());
        controller.x()
            .onTrue(arm.scoreMidConeC());
        controller.y()
            .onTrue(arm.scoreHighConeC());
            
        controller.povDown()
            .onTrue(arm.inC());
        controller.povRight()
            .onTrue(arm.coneInC());
        controller.povUp()
            .onTrue(arm.pickUpDoubleSubC());
        controller.povLeft()
            .onTrue(arm.pickUpSingleSubC());
        controller.leftBumper()
            .whileTrue(run(()->arm.setShoulderPosRadians(arm.shoulderPid.getSetpoint().position-0.05)));
        controller.rightBumper()
            .whileTrue(run(()->arm.setShoulderPosRadians(arm.shoulderPid.getSetpoint().position+0.05)));

        controller.start()
            .onTrue(arm.pickUpGroundC());
        controller.back()
            .onTrue(arm.pickUpCubeGroundC());
            
    }
 
    public CommandBase getAutoCommand(){
        return autoOptions.getAutoCommand();
    }

    public void log() {
        Logger.updateEntries();
        drive.log();

        arm.log();
        SmartDashboard.putNumber("Robot/Pressure", compressor.getPressure());
    }

    public void simulationPeriodic(){
        // visionSim.processFrame(drive.getPose());
    }
}

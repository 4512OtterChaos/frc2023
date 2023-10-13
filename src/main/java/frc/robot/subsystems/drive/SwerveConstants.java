package frc.robot.subsystems.drive;



import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class SwerveConstants {

    public static final double kLinearAcceleration = Units.feetToMeters(20); //m/s/s
    public static final double kLinearDeceleration = Units.feetToMeters(30);
    public static final double kRotationalAcceleration = Units.rotationsToRadians(6);
    public static final double kRotationalDeceleration = Units.rotationsToRadians(10);

    private static final boolean isReal = RobotBase.isReal();
    public static final int kPigeonID = 0;

    // Inversions
    public static final boolean kInvertGyro = false;
    public static final boolean kInvertDrive = false;
    public static final boolean kInvertSteer = false;
    public static final boolean kInvertCancoder = false;
    // Physical properties
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    public static final double kTrackLength = Units.inchesToMeters(21.5);
    
    public static final double kMaxLinearSpeed = Units.feetToMeters(15.5);
    public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kWheelCircumference = kWheelDiameter*Math.PI;
    public static final double kDriveGearRatio = 6.12; // 6.12:1 L3 ratio
    public static final double kSteerGearRatio = 12.8; // 12.8:1

    public enum Module {
        FL(1, 7, 3, 3, -8.525, kTrackLength/2, kTrackWidth/2), // Front left
        FR(2, 2, 6, 1, 103.799, kTrackLength/2, -kTrackWidth/2), // Front Right
        BL(3, 1, 5, 0, 55.107, -kTrackLength/2, kTrackWidth/2), // Back Left
        BR(4, 4, 0, 2, 65.215, -kTrackLength/2, -kTrackWidth/2); // Back Right

        public final int moduleNum;
        public final int driveMotorID;
        public final int steerMotorID;
        public final int cancoderID;
        public final double angleOffset;
        public final Translation2d centerOffset;
        private Module(int moduleNum, int driveMotorID, int steerMotorID, int cancoderID, double angleOffset, double xOffset, double yOffset){
            this.moduleNum = moduleNum;
            this.driveMotorID = driveMotorID;
            this.steerMotorID = steerMotorID;
            this.cancoderID = cancoderID;
            this.angleOffset = angleOffset;
            centerOffset = new Translation2d(xOffset, yOffset);
        }
    }

    // Current limits
    public static final double kDriveContinuousCurrentLimit = 40;
    public static final int kDrivePeakCurrentLimit = 65;
    public static final double kDrivePeakCurrentDuration = 0.1;
    public static final int kSteerContinuousCurrentLimit = 25;
    public static final int kSteerPeakCurrentLimit = 40;
    public static final double kSteerPeakCurrentDuration = 0.1;
    // Voltage compensation
    public static final double kVoltageSaturation = 12;
    public static final int kVoltageMeasurementSamples = 32;

    // Feedforward
    // Linear drive feed forward
    public static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward( // real
        0.2, // Voltage to break static friction
        2.25, // Volts per meter per second
        0.17 // Volts per meter per second squared
    );
    // Steer feed forward
    public static final SimpleMotorFeedforward kSteerFF = new SimpleMotorFeedforward( // real
        0.55, // Voltage to break static friction
        0.23, // Volts per radian per second
        0.0056 // Volts per radian per second squared
    );

    // PID
    public static final double kDriveKP = 0.025;
    public static final double kDriveKI = 0;
    public static final double kDriveKD = 0;

    public static final double kSteerKP = 0.4;
    public static final double kSteerKI = 0;
    public static final double kSteerKD = 0.5;
    public static final double kSteerVelocity = 8; // rotations per second
    public static final double kSteerAcceleration = 40; // rotations per second squared
    public static final int kAllowableSteeringError = 80;

    // The configurations applied to swerve CTRE devices
    public static final Slot0Configs drivePIDConfig = new Slot0Configs();
    public static final CurrentLimitsConfigs driveCurrentLimitsConfig = new CurrentLimitsConfigs();
    public static final CurrentLimitsConfigs driveCurrentLimitsConfig = new CurrentLimitsConfigs();
    public static final Slot0Configs steerConfig = new Slot0Configs();
    public static final Slot0Configs cancoderConfig = new Slot0Configs();
    public static final Slot0Configs kPigeon2Config = new Slot0Configs();
    public static final int kCANTimeout = 100;

    static {
        drivePIDConfig.kP = kDriveKP;
        drivePIDConfig.kI = kDriveKI;
        drivePIDConfig.kD = kDriveKD;
        driveCurrentLimitsConfig.StatorCurrentLimit = kDriveContinuousCurrentLimit;
        driveCurrentLimitsConfig.StatorCurrentLimitEnable = true;

        driveConfig.voltageCompSaturation = kVoltageSaturation;
        driveConfig.voltageMeasurementFilter = kVoltageMeasurementSamples;
        driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_20Ms;
        driveConfig.velocityMeasurementWindow = 32;

        steerConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        steerConfig.kP = kSteerKP;
        steerConfig.kI = kSteerKI;
        steerConfig.kD = kSteerKD;
        steerConfig.kF = kSteerFF.kv;
        steerConfig.motionCruiseVelocity = TalonUtil.rotationsToVelocity(kSteerVelocity, kSteerGearRatio);
        steerConfig.motionAcceleration = TalonUtil.rotationsToVelocity(kSteerAcceleration, kSteerGearRatio);
        steerConfig.allowableClosedloopError = kAllowableSteeringError;
        steerConfig.neutralDeadband = isReal ? 0.01 : 0.001;
        steerConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true,
            kSteerContinuousCurrentLimit,
            kSteerPeakCurrentLimit,
            kSteerPeakCurrentDuration
        );
        steerConfig.voltageCompSaturation = kVoltageSaturation;
        steerConfig.voltageMeasurementFilter = kVoltageMeasurementSamples;
        steerConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_20Ms;
        steerConfig.velocityMeasurementWindow = 32;

        cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        cancoderConfig.sensorDirection = kInvertCancoder;
    }
}

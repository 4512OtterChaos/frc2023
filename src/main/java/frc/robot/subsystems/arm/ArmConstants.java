package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class ArmConstants{

    public static final double kRampRate = 0.02;

    //----- Shoulder
    public static final double kShoulderGearing = ((58.0 / 18.0) * (48.0 / 24.0) * 3 * 5);
    public static final Rotation2d kShoulderAngleOffset = new Rotation2d();

    public static final int kShoulderStallLimit = 40;
    public static ArmFeedforward kShoulderFF = new ArmFeedforward(
        0, // Voltage to break static friction
        0.75, // Voltage to counteract gravity
        2, // Volts per radian per second
        3 // Volts per radian per second squared
    );
    public static final double kShoulderka = kShoulderFF.ka;
    public static final double kShoulderkg = kShoulderFF.kg;
    public static final double kShoulderks = kShoulderFF.ks;
    public static final double kShoulderkv = kShoulderFF.kv;

    public static final double kShoulderKP = 6;
    public static final double kShoulderKI = 0;
    public static final double kShoulderKD = 0;
    public static final double kShoulderVelocityDeg = 90;
    public static final double kShoulderAccelDeg = 90;
    public static final double kShoulderPosToleranceDeg = 5;
    public static final double kShoulderVelToleranceDeg = 10;


    //----- Wrist
    public static final Rotation2d kWristAngleOffset = new Rotation2d();
    public static final double kWristGearing = ((22.0 / 16.0) * 4 * 4 * 5);

    public static final int kWristStallLimit = 30;
    public static ArmFeedforward kWristFF = new ArmFeedforward(
        0, // Voltage to break static friction
        1.4, // Voltage to counteract gravity
        5, // Volts per radian per second
        0.1 // Volts per radian per second squared
    );

    public static final double kWristka = kWristFF.ka;
    public static final double kWristkg = kWristFF.kg;
    public static final double kWristks = kWristFF.ks;
    public static final double kWristkv = kWristFF.kv;

    public static final double kWristKP = 6;
    public static final double kWristKI = 0;
    public static final double kWristKD = 0;
    public static final double kWristVelocityDeg = 90;
    public static final double kWristAccelDeg = 90;
    public static final double kWristPosToleranceDeg = 5;
    public static final double kWristVelToleranceDeg = 10;


    //----- Safety angles
    public static final double kWristMinimumDeg = -30; // equal to Units.degreesToRadians(-45)
    public static final double kWristMaximumDeg = 90; // equal to Units.degreesToRadians(45)

    public static final double kShoulderMinimumDeg = -90; // equal to Units.degreesToRadians(-90)
    public static final double kShoulderMaximumDeg = 30; // equal to Units.degreesToRadians(30);

    public static final double kShoulderExtensionWristLevel = -0.3; // equal to Units.degreesToRadians(-17.1887);
    public static final double kShoulderWristLevel = -0.6; // equal to Units.degreesToRadians(-34.3775);
    public static final double kShoulderMinimumAngleWrist = -0.75; // equal to Units.degreesToRadians(-42.97183);
    public static final double kShoulderMinimumAngleExtension = -0.907571; // equal to Units.degreesToRadians(-52);
    public static final double kShoulderMinimumAngleExtensionWrist = -0.4; // equal to Units.degreesToRadians(-22.9183);
    
}

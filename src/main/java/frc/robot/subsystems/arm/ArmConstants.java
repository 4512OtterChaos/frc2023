package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public final class ArmConstants{

    public static final double kRampRate = 0.02;

    //----- Shoulder
    public static final double kShoulderGearing = ((58.0 / 18.0) * (48.0 / 24.0) * 3 * 5);
    public static final Rotation2d kShoulderAngleOffset = Rotation2d.fromDegrees(53.75);

    public static final double kShoulderks = 0.001;
    public static final double kShoulderkg = 1;
    public static final double kShoulderkv = 0.001;
    public static final double kShoulderka = 0.001;
    public static ArmFeedforward kShoulderFF = new ArmFeedforward(
        kShoulderks, // Voltage to break static friction
        kShoulderkg, // Voltage to counteract gravity
        kShoulderkv, // Volts per radian per second
        kShoulderka // Volts per radian per second squared
    );
    public static final int kShoulderStallLimit = 40;

    public static final double kShoulderKP = 0;
    public static final double kShoulderKI = 0;
    public static final double kShoulderKD = 0;
    public static final double kShoulderVelocityDeg = 90;
    public static final double kShoulderAccelDeg = 90;
    public static final double kShoulderPosToleranceDeg = 5;
    public static final double kShoulderVelToleranceDeg = 10;


    //----- Wrist
    public static final Rotation2d kWristAngleOffset = Rotation2d.fromDegrees(9);
    public static final double kWristGearing = ((22.0 / 16.0) * 4 * 4 * 5);

    public static final double kWristks = 0.001;
    public static final double kWristkg = 0.25;
    public static final double kWristkv = 0.001;
    public static final double kWristka = 0.001;
    public static ArmFeedforward kWristFF = new ArmFeedforward(
        kWristks, // Voltage to break static friction
        kWristkg, // Voltage to counteract gravity
        kWristkv, // Volts per radian per second
        kWristka // Volts per radian per second squared
    );
    public static final int kWristStallLimit = 30;

    public static final double kWristKP = 0;
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

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.ArmLerpFeedforward;

public final class ArmConstants{

    public static final double kRampRate = 0.02;

    //----- Shoulder
    public static final double kShoulderGearing = ((58.0 / 18.0) * (48.0 / 24.0) * 3 * 5);
    public static final Rotation2d kShoulderAngleOffset = Rotation2d.fromDegrees(41);

    // Retracted feedforward
    public static final double kShoulderks1 = 0.001;
    public static final double kShoulderkg1 = 0.6;
    public static final double kShoulderkv1 = 0.001;
    public static final double kShoulderka1 = 0.001;
    // Extended feedforward
    public static final double kShoulderks2 = 0.001;
    public static final double kShoulderkg2 = 1;
    public static final double kShoulderkv2 = 0.001;
    public static final double kShoulderka2 = 0.001;
    public static ArmLerpFeedforward kShoulderFF = new ArmLerpFeedforward(
        kShoulderks1, // Voltage to break static friction
        kShoulderkg1, // Voltage to counteract gravity
        kShoulderkv1, // Volts per radian per second
        kShoulderka1, // Volts per radian per second squared

        kShoulderks2,
        kShoulderkg2,
        kShoulderkv2,
        kShoulderka2
    );
    public static final int kShoulderStallLimit = 40;

    public static final double kShoulderKP = 14;
    public static final double kShoulderKI = 0;
    public static final double kShoulderKD = 0.5;
    public static final double kShoulderVelocityDeg = 135;
    public static final double kShoulderAccelDeg = 180;
    public static final double kShoulderPosToleranceDeg = 5;
    public static final double kShoulderVelToleranceDeg = 30;


    //----- Wrist
    public static final Rotation2d kWristAngleOffset = Rotation2d.fromDegrees(185);
    public static final double kWristGearing = ((22.0 / 16.0) * 4 * 4 * 5);

    public static final double kWristks = 0.001;
    public static final double kWristkg = 0.3;
    public static final double kWristkv = 0.001;
    public static final double kWristka = 0.001;
    public static ArmFeedforward kWristFF = new ArmFeedforward(
        kWristks, // Voltage to break static friction
        kWristkg, // Voltage to counteract gravity
        kWristkv, // Volts per radian per second
        kWristka // Volts per radian per second squared
    );
    public static final int kWristStallLimit = 25;

    public static final double kWristKP = 8;
    public static final double kWristKI = 0;
    public static final double kWristKD = 0.5;
    public static final double kWristVelocityDeg = 250;
    public static final double kWristAccelDeg = 400;
    public static final double kWristPosToleranceDeg = 5;
    public static final double kWristVelToleranceDeg = 30;


    //----- Safety angles
    public static final double kWristMinimumDeg = -35; // Absolute minimum wrist angle
    public static final double kWristMaximumDeg = 89; // Absolute maximum wrist angle

    public static final double kShoulderMinimumDeg = -90; // Absolute minimum shoulder angle
    public static final double kShoulderMaximumDeg = 30; // Absolute maximum shoulder angle

    public static final double kShoulderMinWristDownDeg = -55; // Minimum shoulder angle when retracted and wrist is down
    public static final double kShoulderMinExtendedDeg = -50; // Minimum shoulder angle when extended and wrist is level
    public static final double kShoulderMinExtendedWristDownDeg = -32; // Minimum shoulder angle when extended and wrist is down
}

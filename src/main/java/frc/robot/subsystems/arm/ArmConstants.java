package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class ArmConstants{

    public static  ArmFeedforward kShoulderFF= new ArmFeedforward(
        0, // Voltage to break static friction
        1.5, // Voltage to counteract gravity
        3, // Volts per meter per second
        0.1 // Volts per meter per second squared
    );
    public static final double kShoulderka = kShoulderFF.ka;
    public static final double kShoulderkg = kShoulderFF.kg;
    public static final double kShoulderks = kShoulderFF.ks;
    public static final double kShoulderkv = kShoulderFF.kv;
    
    public static   ArmFeedforward kWristFF= new ArmFeedforward(
        0, // Voltage to break static friction
        0, // Voltage to counteract gravity
        5, // Volts per meter per second
        0.1 // Volts per meter per second squared
    );
    public static final double kWristka = kShoulderFF.ka;
    public static final double kWristkg = 1.4;
    public static final double kWristks = kShoulderFF.ks;
    public static final double kWristkv = kShoulderFF.kv;

    public static final double kWristMinimumAngle = -0.785398; // equal to Units.degreesToRadians(-45)
    public static final double kWristMaximumAngle = 0.785398; // equal to Units.degreesToRadians(45)

    public static final double kShoulderExtensionWristLevel = -0.3; // equal to Units.degreesToRadians(-17.1887);
    public static final double kShoulderWristLevel = -0.6; // equal to Units.degreesToRadians(-34.3775);
    

    public static final double kShoulderMinimumAngle = -1.5708; // equal to Units.degreesToRadians(-90)
    public static final double kShoulderMinimumAngleWrist = -0.75; // equal to Units.degreesToRadians(-42.97183);
    public static final double kShoulderMinimumAngleExtension = -0.907571; // equal to Units.degreesToRadians(-52);
    public static final double kShoulderMinimumAngleExtensionWrist = -0.4; // equal to Units.degreesToRadians(-22.9183);
    public static final double kShoulderMaximumAngle = 0.523599; // equal to Units.degreesToRadians(30);

    public static final Rotation2d kShoulderAngleOffset = new Rotation2d();
    
    public static final Rotation2d kWristAngleOffset = new Rotation2d();

    public static final int kShoulderContinuousLimit = 40;
    public static final int kShoulderPeakLimit = 50;

    public static final int kWristContinuousLimit = 30;
    public static final int kWristPeakLimit = 35;

    public static final double kRampRate = 0.02;
    
}

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;

public final class ArmConstants{

    public static final int kEncoderPPR = 4096;

    public static final double kTravelHeight = Units.inchesToMeters(82.5);
    // measured encoder pulses at max elevator height
    public static final int kTravelPulses = 4300;
    public static final double kEncoderDistPerPulse = kTravelHeight / kTravelPulses;

    public static  ArmFeedforward kShoulderFF= new ArmFeedforward(
        0, // Voltage to break static friction
        1.5, // Voltage to counteract gravity
        3, // Volts per meter per second
        0.1 // Volts per meter per second squared
    );
    public static  ArmFeedforward kWristFF= new ArmFeedforward(
        0, // Voltage to break static friction
        1.4, // Voltage to counteract gravity
        5, // Volts per meter per second
        0.1 // Volts per meter per second squared
    );

    public static final double kWristMinimumAngle = -1.0472; // equal to Units.degreesToRadians(-60)
    public static final double kWristMaximumAngle = 0.785398; // equal to Units.degreesToRadians(45)

    public static final double kShoulderExstensionWristLevel = -0.65; // equal to Units.degreesToRadians(-47);
    public static final double kShoulderWristLevel = -0.7; // equal to Units.degreesToRadians(-70);
    

    public static final double kShoulderMinimumAngle = -1.5708; // equal to Units.degreesToRadians(-90)
    public static final double kShoulderMinimumAngleWrist = -1; // equal to Units.degreesToRadians(-80);
    public static final double kShoulderMinimumAngleExtension = -0.907571; // equal to Units.degreesToRadians(-58);
    public static final double kShoulderMinimumAngleExtensionWrist = -0.698132; // equal to Units.degreesToRadians(-40);
    public static final double kShoulderMaximumAngle = 0.523599; // equal to Units.degreesToRadians(30);
}

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
        3, // Voltage to counteract gravity
        3, // Volts per meter per second
        0.1 // Volts per meter per second squared
    );
    public static  ArmFeedforward kWristFF= new ArmFeedforward(
        0, // Voltage to break static friction
        0.5, // Voltage to counteract gravity
        2, // Volts per meter per second
        0.05 // Volts per meter per second squared
    );
}

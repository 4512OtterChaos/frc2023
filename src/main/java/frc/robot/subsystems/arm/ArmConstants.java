package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.annotations.Config;

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

    public static final double kWristMinimumAngle = Units.degreesToRadians(-45);
    public static final double kWristMaximumAngle = Units.degreesToRadians(45);

    public static final double kShoulderMinimumAngle = Units.degreesToRadians(-90);
    public static final double kShoulderMinimumAngleExtension = Units.degreesToRadians(-40);
    public static final double kShoulderMaximumAngle = Units.degreesToRadians(30);
}

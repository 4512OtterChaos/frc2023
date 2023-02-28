package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.DutyCycleEncoderSim;
import frc.robot.util.OCSparkMax;
import frc.robot.util.PistonSim;
import frc.robot.util.VariableLengthArmSim;

import static frc.robot.subsystems.arm.ArmConstants.*;

public class ArmSimulation {

    private final OCSparkMax shoulderMotor;
    private final OCSparkMax wristMotor;

    private final DutyCycleEncoder shoulderEncoder;
    private final DutyCycleEncoder wristEncoder;
    private final DutyCycleEncoderSim shoulderEncoderSim;
    private final DutyCycleEncoderSim wristEncoderSim;

    private final DoubleSolenoid extensionPiston;

    private final double kWristMassKg = Units.lbsToKilograms(8);
    private final double kArmMassKg = Units.lbsToKilograms(7) + kWristMassKg; // INCLUDING wrist

    private final double kShoulderHeight = Units.inchesToMeters(40.75);
    private final double kBaseStageLength = Units.inchesToMeters(26.25);
    private final double kRetractedFirstStageLength = Units.inchesToMeters(4);
    private final double kExtensionLength = Units.inchesToMeters(18);
    private final double kWristLength = Units.inchesToMeters(16);

    private final VariableLengthArmSim shoulderSim = new VariableLengthArmSim(
            LinearSystemId.identifyPositionSystem(
                    kShoulderFF.kv,
                    kShoulderFF.ka),
            DCMotor.getNEO(2),
            ((58.0 / 18.0) * (52.0 / 20.0) * 3 * 5),
            VariableLengthArmSim.estimateMOI(kBaseStageLength, kArmMassKg),
            kBaseStageLength + kWristLength,
            Units.degreesToRadians(-90),
            Units.degreesToRadians(30),
            kArmMassKg,
            true);

    private final VariableLengthArmSim wristSim = new VariableLengthArmSim(
            LinearSystemId.identifyPositionSystem(
                    kWristFF.kv,
                    kWristFF.ka),
            DCMotor.getNeo550(1),
            ((22.0 / 16.0) * 4 * 4 * 5),
            VariableLengthArmSim.estimateMOI(kBaseStageLength, kArmMassKg),
            kWristLength,
            Units.degreesToRadians(-45),
            Units.degreesToRadians(90),
            kWristMassKg,
            true);

    private final PistonSim pistonSim = new PistonSim(false, 0.5);

    public ArmSimulation(
            OCSparkMax shoulderMotor, OCSparkMax wristMotor, DoubleSolenoid extensionPiston,
            DutyCycleEncoder shoulderEncoder, DutyCycleEncoder wristEncoder) {
        this.shoulderMotor = shoulderMotor;
        this.wristMotor = wristMotor;
        this.extensionPiston = extensionPiston;
        this.shoulderEncoder = shoulderEncoder;
        this.wristEncoder = wristEncoder;
        shoulderEncoderSim = new DutyCycleEncoderSim(shoulderEncoder);
        wristEncoderSim = new DutyCycleEncoderSim(wristEncoder);

        SmartDashboard.putData("Arm/Mech2d", mech);
    }

    // Ligaments of arm simulation, every ligament root must extend off of another.
    Mechanism2d mech = new Mechanism2d(2.5, 2.5, new Color8Bit(0, 100, 150));
    MechanismRoot2d mechRoot = mech.getRoot("arm", 1.25, 0.1);

    private final Color8Bit kSetpointBaseColor = new Color8Bit(150, 0, 0);
    private final Color8Bit kSetpointExtensionColor = new Color8Bit(180, 0, 0);
    private final Color8Bit kMechBaseColor = new Color8Bit(0, 0, 150);
    private final Color8Bit kMechExtensionColor = new Color8Bit(0, 0, 180);

    private final double kSetpointWidth = 6;
    private final double kMechWidth = 9;

    private final double kDefaultShoulderDeg = -180;
    private final double kDefaultWristDeg = 90;

    MechanismLigament2d mechArmBase = mechRoot.append(
            new MechanismLigament2d("base", kShoulderHeight, 90, kMechWidth, kMechBaseColor));
    MechanismLigament2d mechArm = mechArmBase.append(
            new MechanismLigament2d("arm", kBaseStageLength, kDefaultShoulderDeg, kMechWidth, kMechBaseColor));
    MechanismLigament2d mechExtension = mechArm.append(
            new MechanismLigament2d("armExtension", kRetractedFirstStageLength, 0, kMechWidth - 2,
                    kMechExtensionColor));
    MechanismLigament2d mechWrist = mechExtension.append(
            new MechanismLigament2d("armWrist", kWristLength, kDefaultWristDeg, kMechWidth + 1, kMechExtensionColor));

    MechanismLigament2d setpointArmBase = mechRoot.append(
            new MechanismLigament2d("setpointArmBase", kShoulderHeight, 90, kSetpointWidth, kSetpointBaseColor));
    MechanismLigament2d setpointArm = setpointArmBase.append(
            new MechanismLigament2d("setpointArm", kBaseStageLength, kDefaultShoulderDeg, kSetpointWidth,
                    kSetpointBaseColor));
    MechanismLigament2d setpointExtension = setpointArm.append(
            new MechanismLigament2d("setpointExtension", kRetractedFirstStageLength, 0, kSetpointWidth - 2,
                    kSetpointExtensionColor));
    MechanismLigament2d setpointWrist = setpointExtension.append(
            new MechanismLigament2d("setpointWrist", kWristLength, kDefaultWristDeg, kSetpointWidth + 1,
                    kSetpointExtensionColor));

    private double shoulderSetpointRadians = -Math.PI;
    private double wristSetpointRadians = Math.PI;

    /** 
     * @param radians Frame-relative shoulder angle in radians
     */
    public void setShoulderSetpoint(double radians) {
        shoulderSetpointRadians = radians;
    }

    /**
     * @param radians Frame-relative wrist angle in radians
     */
    public void setWristSetpoint(double radians) {
        wristSetpointRadians = radians;
    }

    /**
     * Get the extension length of this piston (0 - 1).
     */
    public double getExtension() {
        return pistonSim.getExtension();
    }

    /**
     * Call this method periodically, always.
     */
    public void periodic() {
        mechArm.setAngle(-90 + Units.radiansToDegrees(shoulderEncoder.getDistance()));
        mechWrist.setAngle(Units.radiansToDegrees(wristEncoder.getDistance()));

        setpointArm.setAngle(-90 + Units.radiansToDegrees(shoulderSetpointRadians));
        setpointWrist.setAngle(Units.radiansToDegrees(wristSetpointRadians - shoulderSetpointRadians));

        pistonSim.setExtended(extensionPiston.get() == Value.kForward);
        pistonSim.update();
        double extensionLength = kRetractedFirstStageLength + kExtensionLength * pistonSim.getExtension();
        mechExtension.setLength(extensionLength);
        setpointExtension.setLength(extensionLength);
    }

    /**
     * Call this method periodically during simulation.
     */
    public void simulationPeriodic() {
        // dynamically change our arm's center of gravity and moment of inertia
        double armLength = kBaseStageLength + kRetractedFirstStageLength;
        armLength += getExtension() * kExtensionLength;
        armLength += Math.cos(wristSim.getAngleRads() - shoulderSim.getAngleRads()) * kWristLength;
        shoulderSim.setCGRadius(armLength / 2.0);
        shoulderSim.setMOI(VariableLengthArmSim.estimateMOI(armLength, kArmMassKg));

        // get "voltage" after static friction
        double shoulderVoltage = shoulderMotor.getAppliedOutput();
        double wristVoltage = wristMotor.getAppliedOutput();

        // if(voltage >= 0) voltage = Math.max(0, voltage-kFF.ks);
        // else voltage = Math.min(0, voltage+kFF.ks);

        // apply this voltage to the simulated physics model
        shoulderSim.setInput(shoulderVoltage);
        shoulderSim.update(0.02);

        wristSim.setInput(wristVoltage);
        wristSim.update(0.02);

        // update our sensors with the results
        shoulderEncoderSim.setDistance(shoulderSim.getAngleRads());
        wristEncoderSim.setDistance(wristSim.getAngleRads() - shoulderSim.getAngleRads());
    }
}

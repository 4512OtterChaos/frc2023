package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.OCSparkMax;
import frc.robot.util.sim.DutyCycleEncoderSim;
import frc.robot.util.sim.PistonSim;
import frc.robot.util.sim.VariableLengthArmSim;

import static frc.robot.subsystems.arm.ArmConstants.*;

import java.util.Optional;

public class ArmSimulation {

    private final Arm arm;
    private final OCSparkMax shoulderMotor;
    private final OCSparkMax wristMotor;

    private final DutyCycleEncoderSim shoulderEncoderSim;
    private final DutyCycleEncoderSim wristEncoderSim;

    private static final double kWristMassKg = Units.lbsToKilograms(8);
    private static final double kArmMassKg = Units.lbsToKilograms(7) + kWristMassKg; // INCLUDING wrist

    private static final double kShoulderHeight = Units.inchesToMeters(40.75);
    private static final double kBaseStageLength = Units.inchesToMeters(26.25);
    private static final double kRetractedFirstStageLength = Units.inchesToMeters(4);
    private static final double kExtensionLength = Units.inchesToMeters(18);
    private static final double kWristLength = Units.inchesToMeters(15);

    private final VariableLengthArmSim shoulderSim = new VariableLengthArmSim(
            LinearSystemId.identifyPositionSystem(
                    kShoulderFF.kv1,
                    kShoulderFF.ka1),
            DCMotor.getNEO(2),
            kShoulderGearing,
            VariableLengthArmSim.estimateMOI(kBaseStageLength, kArmMassKg),
            kBaseStageLength + kWristLength,
            Units.degreesToRadians(kShoulderMinimumDeg),
            Units.degreesToRadians(kShoulderMaximumDeg),
            kArmMassKg,
            true);

    private final VariableLengthArmSim wristSim = new VariableLengthArmSim(
            LinearSystemId.identifyPositionSystem(
                    kWristFF.kv,
                    kWristFF.ka),
            DCMotor.getNeo550(1),
            kWristGearing,
            // VariableLengthArmSim.estimateMOI(kWristLength, kWristMassKg),
            VariableLengthArmSim.estimateMOI(kBaseStageLength, kArmMassKg), // TODO: figure out why this is broken
            kWristLength,
            Units.degreesToRadians(kWristMinimumDeg),
            Units.degreesToRadians(kWristMaximumDeg),
            kWristMassKg,
            true);

    public ArmSimulation(
            Arm arm, OCSparkMax shoulderMotor, OCSparkMax wristMotor,
            DutyCycleEncoder shoulderEncoder, DutyCycleEncoder wristEncoder) {
        this.arm = arm;
        this.shoulderMotor = shoulderMotor;
        this.wristMotor = wristMotor;
        shoulderEncoderSim = new DutyCycleEncoderSim(shoulderEncoder);
        shoulderEncoderSim.setDistancePerRotation(2 * Math.PI);
        wristEncoderSim = new DutyCycleEncoderSim(wristEncoder);
        wristEncoderSim.setDistancePerRotation(2 * Math.PI);

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

    private final MechanismLigament2d mechArmBase = mechRoot.append(
            new MechanismLigament2d("base", kShoulderHeight, 90, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d mechArm = mechArmBase.append(
            new MechanismLigament2d("arm", kBaseStageLength, kDefaultShoulderDeg, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d mechExtension = mechArm.append(
            new MechanismLigament2d("armExtension", kRetractedFirstStageLength, 0, kMechWidth - 2,
                    kMechExtensionColor));
    private final MechanismLigament2d mechWrist = mechExtension.append(
            new MechanismLigament2d("armWrist", kWristLength, kDefaultWristDeg, kMechWidth + 1, kMechExtensionColor));

    private final MechanismLigament2d setpointArmBase = mechRoot.append(
            new MechanismLigament2d("setpointArmBase", kShoulderHeight, 90, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d setpointArm = setpointArmBase.append(
            new MechanismLigament2d("setpointArm", kBaseStageLength, kDefaultShoulderDeg, kSetpointWidth,
                    kSetpointBaseColor));
    private final MechanismLigament2d setpointExtension = setpointArm.append(
            new MechanismLigament2d("setpointExtension", kRetractedFirstStageLength, 0, kSetpointWidth - 2,
                    kSetpointExtensionColor));
    private final MechanismLigament2d setpointWrist = setpointExtension.append(
            new MechanismLigament2d("setpointWrist", kWristLength, kDefaultWristDeg, kSetpointWidth + 1,
                    kSetpointExtensionColor));

    /**
     * Visualize the current arm state to the Mechanism2d.
     * This method should be called periodically.
     * 
     * @param shoulderPosRadians Ground-relative shoulder position in radians
     * @param wristPosRadians Shoulder-relative wrist position in radians
     * @param extended Whether the arm is (attempting to be) extended or not
     */
    public void visualizeState(double shoulderPosRadians, double wristPosRadians, boolean extended) {
        mechArm.setAngle(-90 + Units.radiansToDegrees(shoulderPosRadians));
        mechWrist.setAngle(Units.radiansToDegrees(wristPosRadians));

        double extensionLength = kRetractedFirstStageLength + kExtensionLength * arm.getExtension();
        mechExtension.setLength(extensionLength);
    }
    /**
     * Visualize the desired arm state to the Mechanism2d.
     * 
     * @param shoulderPosRadians Ground-relative shoulder position in radians
     * @param wristPosRadians Shoulder-relative wrist position in radians
     * @param extended Whether the arm is (attempting to be) extended or not
     */
    public void visualizeSetpoint(double shoulderPosRadians, double wristPosRadians, boolean extended) {
        setpointArm.setAngle(-90 + Units.radiansToDegrees(shoulderPosRadians));
        setpointWrist.setAngle(Units.radiansToDegrees(wristPosRadians));

        setpointExtension.setLength(kRetractedFirstStageLength + (extended ? kExtensionLength : 0));
    }

    /**
     * Call this method periodically during simulation.
     */
    public void simulationPeriodic() {
        // dynamically change our arm's center of gravity and moment of inertia
        double armLength = kBaseStageLength + kRetractedFirstStageLength;
        armLength += arm.getExtension() * kExtensionLength;
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

        wristSim.setGravityAngle(-Math.PI/2.0 - shoulderSim.getAngleRads());
        wristSim.setInput(wristVoltage);
        wristSim.update(0.02);

        // update our sensors with the results
        shoulderEncoderSim.setDistance(shoulderSim.getAngleRads());
        wristEncoderSim.setDistance(wristSim.getAngleRads());
    }

    //----- Kinematics
    private static final Translation2d kShoulderTrl = new Translation2d(0, kShoulderHeight);
    public static Translation2d getWristEndXZ(double shoulderPosRadians, double extension, double wristPosRadians) {
        return getWristPivotXZ(shoulderPosRadians, extension)
                .plus(getWristEndRelXZ(wristPosRadians).rotateBy(new Rotation2d(shoulderPosRadians)));
    }

    public static Translation2d getWristPivotXZ(double shoulderPosRadians, double extension) {
        return new Translation2d(
                kBaseStageLength + kRetractedFirstStageLength + extension * kExtensionLength,
                shoulderPosRadians).plus(kShoulderTrl);
    }

    public static Translation2d getWristEndRelXZ(double wristPosRadians) {
        return new Translation2d(kWristLength, wristPosRadians);
    }

    public static Optional<Double> getShoulderRadsMovedY(Translation2d point, double amountY) {
        double length = point.getDistance(kShoulderTrl);
        double newPointY = point.minus(kShoulderTrl).getY() + amountY;
        if(Math.abs(newPointY) > length) return Optional.empty();
        return Optional.of(Math.asin(newPointY / length));
    }
    public static Optional<Double> getShoulderRadsMovedX(Translation2d point, double amountX) {
        double length = point.getDistance(kShoulderTrl);
        
        double newPointX = point.minus(kShoulderTrl).getX() + amountX;
        if(Math.abs(newPointX) > length) return Optional.empty();
        double newRads = -Math.acos(newPointX / length);
        double pointRads = point.getAngle().getRadians();
        if(pointRads > 0) newRads *= -1;
        return Optional.of(newRads);
    }
}

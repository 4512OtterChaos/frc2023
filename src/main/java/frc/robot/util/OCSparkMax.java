package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotBase;

public class OCSparkMax extends CANSparkMax {

    /**
     * Wraps {@link CANSparkMax#CANSparkMax(int, MotorType)} to provide
     * a {@link OCRelativeEncoder} instead, which has simulation handles for
     * setting position and velocity. 
     * 
     * @param deviceId
     * @param type
     */
    public OCSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    @Override
    public OCRelativeEncoder getEncoder() {
        return new OCRelativeEncoder(super.getEncoder());
    }

    /**
     * Wrapper for {@link RelativeEncoder} which implements handles for
     * setting simulation position and velocity values.
     */
    public static class OCRelativeEncoder implements RelativeEncoder {

        private final RelativeEncoder encoder;
        private double simPosition = 0.0;
        private double simVelocity = 0.0;

        public OCRelativeEncoder(RelativeEncoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getPosition() {
            if(RobotBase.isReal()) {
                return encoder.getPosition();
            }
            else {
                return simPosition;
            }
        }

        @Override
        public double getVelocity() {
            if(RobotBase.isReal()) {
                return encoder.getVelocity();
            }
            else {
                return simVelocity;
            }
        }

        @Override
        public REVLibError setPosition(double position) {
            setSimPosition(position);
            return encoder.setPosition(position);
        }

        public void setSimPosition(double position) {
            simPosition = position;
        }
    
        public void setSimVelocity(double velocity) {
            simVelocity = velocity;
        }

        @Override
        public REVLibError setPositionConversionFactor(double factor) {
            return encoder.setPositionConversionFactor(factor);
        }

        @Override
        public REVLibError setVelocityConversionFactor(double factor) {
            return encoder.setVelocityConversionFactor(factor);
        }

        @Override
        public double getPositionConversionFactor() {
            return encoder.getPositionConversionFactor();
        }

        @Override
        public double getVelocityConversionFactor() {
            return encoder.getVelocityConversionFactor();
        }

        @Override
        public REVLibError setAverageDepth(int depth) {
            return encoder.setAverageDepth(depth);
        }

        @Override
        public int getAverageDepth() {
            return encoder.getAverageDepth();
        }

        @Override
        public REVLibError setMeasurementPeriod(int period_ms) {
            return encoder.setMeasurementPeriod(period_ms);
        }

        @Override
        public int getMeasurementPeriod() {
            return encoder.getMeasurementPeriod();
        }

        @Override
        public int getCountsPerRevolution() {
            return encoder.getCountsPerRevolution();
        }

        @Override
        public REVLibError setInverted(boolean inverted) {
            return encoder.setInverted(inverted);
        }

        @Override
        public boolean getInverted() {
            return encoder.getInverted();
        }
    }
}

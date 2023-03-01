// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/** Class to control a simulated duty cycle encoder. */
public class DutyCycleEncoderSim {
    private SimDouble m_simPosition;
    private SimDouble m_simAbsolutePosition;
    private SimDouble m_simDistancePerRotation;

    /**
     * Constructs from an DutyCycleEncoder object.
     *
     * @param encoder DutyCycleEncoder to simulate
     */
    public DutyCycleEncoderSim(DutyCycleEncoder encoder) {
        if (RobotBase.isSimulation()) {
            SimDeviceSim wrappedSimDevice = new SimDeviceSim(
                    "DutyCycle:DutyCycleEncoder" + "[" + encoder.getSourceChannel() + "]");
            m_simPosition = wrappedSimDevice.getDouble("position");
            m_simAbsolutePosition = wrappedSimDevice.getDouble("absPosition");
            m_simDistancePerRotation = wrappedSimDevice.getDouble("distance_per_rot");
        }
    }

    /**
     * Set the position in turns.
     */
    public void setPosition(double turns) {
        if (RobotBase.isSimulation()) {
            m_simPosition.set(turns);
        }
    }
    /**
     * Set the absolute(0 - 1) position in turns.
     */
    public void setAbsolutePosition(double turns) {
        if (RobotBase.isSimulation()) {
            m_simAbsolutePosition.set(MathUtil.clamp(turns, 0, 1));
        }
    }

    /**
     * Set the distance (affected by distance per rotation).
     */
    public void setDistance(double distance) {
        if (RobotBase.isSimulation()) {
            double position = distance / m_simDistancePerRotation.get();
            setPosition(position);
            setAbsolutePosition(position);
        }
    }

    public void setDistancePerRotation(double distancePerRotation) {
        if(RobotBase.isSimulation()) {
            m_simDistancePerRotation.set(distancePerRotation);
        }
    }
}

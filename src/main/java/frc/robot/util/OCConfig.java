/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/**
 * Used for configuring motor settings.
 */
public final class OCConfig {

    /**
     * Sets following for drivetrain motors and inverts sides.
     * 
     * @param leftMotors
     * @param rightMotors
     * @param isRightInverted
     */
    public static void configureDrivetrain(CANSparkMax[] leftMotors, CANSparkMax[] rightMotors,
            boolean isRightInverted) {
        if (leftMotors.length != rightMotors.length) {
            System.out.println("OCConfig - Invalid drivetrain motors!");
            return;
        }

        for (int i = 0; i < leftMotors.length; i++) {
            if (i > 0) {
                // set followers
                setFollower(leftMotors[0], false, leftMotors[i]);
                setFollower(rightMotors[0], false, rightMotors[i]);
            } else {
                // set leaders
                leftMotors[0].setInverted(!isRightInverted);
                rightMotors[0].setInverted(isRightInverted);
            }
        }
    }

    /**
     * Configures each given motor with given settings.
     * <p>
     * Does not invert nor set followers.
     * 
     * @param motors Array of motors
     */
    public static void configMotors(int stallLimit, int freeLimit, double rampRate, CANSparkMax... motors) {
        for (CANSparkMax motor : motors) {
            // Make sure motor config is clean
            motor.restoreFactoryDefaults();

            // Ramp motors
            motor.setOpenLoopRampRate(rampRate);
            motor.setClosedLoopRampRate(rampRate);

            motor.enableVoltageCompensation(12);

            // Current limits (don't kill the motors)
            if (stallLimit != freeLimit)
                motor.setSmartCurrentLimit(stallLimit, freeLimit);
            else
                motor.setSmartCurrentLimit(stallLimit);
        }
    }

    /**
     * Burns configuration to flash on given motors.
     */
    public static void saveConfig(CANSparkMax... motors) {
        for (CANSparkMax motor : motors) {
            motor.burnFlash();
        }
    }

    /**
     * Sets idle mode of given motors.
     * 
     * @param mode   IdleMode (Brake or Coast)
     * @param motors Motors to set
     */
    public static void setIdleMode(IdleMode mode, CANSparkMax... motors) {
        for (CANSparkMax motor : motors)
            motor.setIdleMode(mode);
    }

    public static void setFollower(CANSparkMax leader, boolean inverted, CANSparkMax... followers) {
        for (CANSparkMax motor : followers) {
            motor.follow(leader, inverted);
        }
        setStatusSlow(followers);
    }

    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
    public static void setStatusFast(CANSparkMax... motors) {
        for (CANSparkMax motor : motors) {
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        }
    }

    public static void setStatusNormal(CANSparkMax... motors) {
        for (CANSparkMax motor : motors) {
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        }
    }

    public static void setStatusSlow(CANSparkMax... motors) {
        for (CANSparkMax motor : motors) {
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        }
    }

    public static void setStatusNothing(CANSparkMax... motors) {
        for (CANSparkMax motor : motors) {
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        }
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * A helper class that computes feedforward outputs for a simple arm (modeled as
 * a motor acting
 * against the force of gravity on a beam suspended at an angle).
 */
public class ArmLerpFeedforward {
    public final double ks1;
    public final double kg1;
    public final double kv1;
    public final double ka1;

    public final double ks2;
    public final double kg2;
    public final double kv2;
    public final double ka2;

    /**
     * Creates a new ArmFeedforward with the specified gains. Units of the gain
     * values will dictate
     * units of the computed feedforward.
     *
     * @param ks1 The first static gain.
     * @param kg1 The first gravity gain.
     * @param kv1 The first velocity gain.
     * @param ka1 The first acceleration gain.
     * @param ks2 The second static gain.
     * @param kg2 The second gravity gain.
     * @param kv2 The second velocity gain.
     * @param ka2 The second acceleration gain.
     */
    public ArmLerpFeedforward(
            double ks1, double kg1, double kv1, double ka1,
            double ks2, double kg2, double kv2, double ka2) {
        this.ks1 = ks1;
        this.kg1 = kg1;
        this.kv1 = kv1;
        this.ka1 = ka1;
        this.ks2 = ks2;
        this.kg2 = kg2;
        this.kv2 = kv2;
        this.ka2 = ka2;
    }

    /**
     * Creates a new ArmFeedforward with the specified gains. Units of the gain
     * values will dictate
     * units of the computed feedforward.
     *
     * @param ks1 The first static gain.
     * @param kg1 The first gravity gain.
     * @param kv1 The first velocity gain.
     * @param ks2 The second static gain.
     * @param kg2 The second gravity gain.
     * @param kv2 The second velocity gain.
     */
    public ArmLerpFeedforward(
            double ks1, double kg1, double kv1,
            double ks2, double kg2, double kv2) {
        this(ks1, kg1, kv1, 0, ks2, kg2, kv2, 0);
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param positionRadians       The position (angle) setpoint. This angle should
     *                              be measured from the
     *                              horizontal (i.e. if the provided angle is 0, the
     *                              arm should be parallel with the floor). If
     *                              your encoder does not follow this convention, an
     *                              offset should be added.
     * @param velocityRadPerSec     The velocity setpoint.
     * @param accelRadPerSecSquared The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double calculate(
            double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared,
            double lerp) {
        double ks = MathHelp.lerp(lerp, ks1, ks2);
        double kg = MathHelp.lerp(lerp, kg1, kg2);
        double kv = MathHelp.lerp(lerp, kv1, kv2);
        double ka = MathHelp.lerp(lerp, ka1, ka2);
        return ks * Math.signum(velocityRadPerSec)
                + kg * Math.cos(positionRadians)
                + kv * velocityRadPerSec
                + ka * accelRadPerSecSquared;
    }

    /**
     * Calculates the feedforward from the gains and velocity setpoint (acceleration
     * is assumed to be
     * zero).
     *
     * @param positionRadians The position (angle) setpoint. This angle should be
     *                        measured from the
     *                        horizontal (i.e. if the provided angle is 0, the arm
     *                        should be parallel with the floor). If
     *                        your encoder does not follow this convention, an
     *                        offset should be added.
     * @param velocity        The velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double positionRadians, double velocity, double lerp) {
        return calculate(positionRadians, velocity, 0, lerp);
    }

    // Rearranging the main equation from the calculate() method yields the
    // formulas for the methods below:

    /**
     * Calculates the maximum achievable velocity given a maximum voltage supply, a
     * position, and an
     * acceleration. Useful for ensuring that velocity and acceleration constraints
     * for a trapezoidal
     * profile are simultaneously achievable - enter the acceleration constraint,
     * and this will give
     * you a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angle        The angle of the arm. This angle should be measured from
     *                     the horizontal (i.e. if
     *                     the provided angle is 0, the arm should be parallel with
     *                     the floor). If your encoder does
     *                     not follow this convention, an offset should be added.
     * @param acceleration The acceleration of the arm.
     * @return The maximum possible velocity at the given acceleration and angle.
     */
    public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration, double lerp) {
        double ks = MathHelp.lerp(lerp, ks1, ks2);
        double kg = MathHelp.lerp(lerp, kg1, kg2);
        double kv = MathHelp.lerp(lerp, kv1, kv2);
        double ka = MathHelp.lerp(lerp, ka1, ka2);
        // Assume max velocity is positive
        return (maxVoltage - ks - Math.cos(angle) * kg - acceleration * ka) / kv;
    }

    /**
     * Calculates the minimum achievable velocity given a maximum voltage supply, a
     * position, and an
     * acceleration. Useful for ensuring that velocity and acceleration constraints
     * for a trapezoidal
     * profile are simultaneously achievable - enter the acceleration constraint,
     * and this will give
     * you a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angle        The angle of the arm. This angle should be measured from
     *                     the horizontal (i.e. if
     *                     the provided angle is 0, the arm should be parallel with
     *                     the floor). If your encoder does
     *                     not follow this convention, an offset should be added.
     * @param acceleration The acceleration of the arm.
     * @return The minimum possible velocity at the given acceleration and angle.
     */
    public double minAchievableVelocity(double maxVoltage, double angle, double acceleration, double lerp) {
        double ks = MathHelp.lerp(lerp, ks1, ks2);
        double kg = MathHelp.lerp(lerp, kg1, kg2);
        double kv = MathHelp.lerp(lerp, kv1, kv2);
        double ka = MathHelp.lerp(lerp, ka1, ka2);
        // Assume min velocity is negative, ks flips sign
        return (-maxVoltage + ks - Math.cos(angle) * kg - acceleration * ka) / kv;
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage
     * supply, a position, and
     * a velocity. Useful for ensuring that velocity and acceleration constraints
     * for a trapezoidal
     * profile are simultaneously achievable - enter the velocity constraint, and
     * this will give you a
     * simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle      The angle of the arm. This angle should be measured from
     *                   the horizontal (i.e. if
     *                   the provided angle is 0, the arm should be parallel with
     *                   the floor). If your encoder does
     *                   not follow this convention, an offset should be added.
     * @param velocity   The velocity of the arm.
     * @return The maximum possible acceleration at the given velocity.
     */
    public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity, double lerp) {
        double ks = MathHelp.lerp(lerp, ks1, ks2);
        double kg = MathHelp.lerp(lerp, kg1, kg2);
        double kv = MathHelp.lerp(lerp, kv1, kv2);
        double ka = MathHelp.lerp(lerp, ka1, ka2);
        return (maxVoltage - ks * Math.signum(velocity) - Math.cos(angle) * kg - velocity * kv) / ka;
    }

    /**
     * Calculates the minimum achievable acceleration given a maximum voltage
     * supply, a position, and
     * a velocity. Useful for ensuring that velocity and acceleration constraints
     * for a trapezoidal
     * profile are simultaneously achievable - enter the velocity constraint, and
     * this will give you a
     * simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle      The angle of the arm. This angle should be measured from
     *                   the horizontal (i.e. if
     *                   the provided angle is 0, the arm should be parallel with
     *                   the floor). If your encoder does
     *                   not follow this convention, an offset should be added.
     * @param velocity   The velocity of the arm.
     * @return The minimum possible acceleration at the given velocity.
     */
    public double minAchievableAcceleration(double maxVoltage, double angle, double velocity, double lerp) {
        return maxAchievableAcceleration(-maxVoltage, angle, velocity, lerp);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.simulation;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;

/** Represents a simulated elevator mechanism. */
public class NewElevatorSim extends NewLinearSystemSim<N2, N1, N1> {
    // Gearbox for the elevator.
    private final DCMotor m_gearbox;

    // The min allowable height for the elevator.
    private final double m_minHeight;

    // The max allowable height for the elevator.
    private final double m_maxHeight;

    // The static friction voltage.
    private final double m_staticFrictionVoltage;

    // The static friction matrix
    private final Matrix<N1, N1> m_staticFriction;

    // The usuable input matrix
    private Matrix<N1, N1> m_usuableInput;

    // The effective gravity during free fall
    private final double m_effectiveGravity;

    /**
     * Creates a simulated elevator mechanism.
     *
     * @param kV                   The velocity gain.
     * @param kA                   The acceleration gain.
     * @param gearbox              The type of and number of motors in the elevator
     *                             gearbox.
     * @param minHeightMeters      The min allowable height of the elevator.
     * @param maxHeightMeters      The max allowable height of the elevator.
     * @param simulateGravity      Whether gravity should be simulated or not.
     * @param startingHeightMeters The starting height of the elevator.
     * @param measurementStdDevs   The standard deviations of the measurements.
     */
    public NewElevatorSim(
            double kV,
            double kA,
            double kG,
            double staticFrictionVoltage,
            DCMotor gearbox,
            double minHeightMeters,
            double maxHeightMeters,
            double startingHeightMeters,
            double... measurementStdDevs) {
        super(
                LinearSystemId.identifyPositionSystem(kV, kA),
                measurementStdDevs);
        m_gearbox = gearbox;
        m_minHeight = minHeightMeters;
        m_maxHeight = maxHeightMeters;
        m_effectiveGravity = kG / kA;
        m_staticFrictionVoltage = staticFrictionVoltage;
        m_staticFriction = new Matrix<>(new SimpleMatrix(1, 1));
        m_usuableInput = new Matrix<>(new SimpleMatrix(1, 1));
        setState(startingHeightMeters, 0);
    }

    /**
     * Creates a simulated elevator mechanism.
     *
     * @param gearbox              The type of and number of motors in the elevator
     *                             gearbox.
     * @param gearing              The gearing of the elevator (numbers greater than
     *                             1 represent reductions).
     * @param carriageMassKg       The mass of the elevator carriage.
     * @param drumRadiusMeters     The radius of the drum that the elevator spool is
     *                             wrapped around.
     * @param minHeightMeters      The min allowable height of the elevator.
     * @param maxHeightMeters      The max allowable height of the elevator.
     * @param startingHeightMeters The starting height of the elevator.
     * @param measurementStdDevs   The standard deviations of the measurements.
     */
    public NewElevatorSim(
            DCMotor gearbox,
            double gearing,
            double carriageMassKg,
            double drumRadiusMeters,
            double minHeightMeters,
            double maxHeightMeters,
            double startingHeightMeters,
            double effectiveGravityMetersPerSecondSq,
            double staticFrictionVoltage,
            double... measurementStdDevs) {
        super(
                LinearSystemId.createElevatorSystem(gearbox, carriageMassKg, drumRadiusMeters, gearing),
                measurementStdDevs);

        m_gearbox = gearbox;
        m_minHeight = minHeightMeters;
        m_maxHeight = maxHeightMeters;
        m_effectiveGravity = effectiveGravityMetersPerSecondSq;
        m_staticFrictionVoltage = staticFrictionVoltage;
        m_staticFriction = new Matrix<>(new SimpleMatrix(1, 1));
        m_usuableInput = new Matrix<>(new SimpleMatrix(1, 1));
        setState(startingHeightMeters, 0);
    }

    /**
     * Sets the elevator's state. The new position will be limited between the
     * minimum and maximum
     * allowed heights.
     *
     * @param positionMeters          The new position in meters.
     * @param velocityMetersPerSecond New velocity in meters per second.
     */
    public final void setState(double positionMeters, double velocityMetersPerSecond) {
        setState(
                VecBuilder.fill(
                        MathUtil.clamp(positionMeters, m_minHeight, m_maxHeight), velocityMetersPerSecond));
    }

    /**
     * Returns whether the elevator would hit the lower limit.
     *
     * @param elevatorHeightMeters The elevator height.
     * @return Whether the elevator would hit the lower limit.
     */
    public boolean wouldHitLowerLimit(double elevatorHeightMeters) {
        return elevatorHeightMeters <= this.m_minHeight;
    }

    /**
     * Returns whether the elevator would hit the upper limit.
     *
     * @param elevatorHeightMeters The elevator height.
     * @return Whether the elevator would hit the upper limit.
     */
    public boolean wouldHitUpperLimit(double elevatorHeightMeters) {
        return elevatorHeightMeters >= this.m_maxHeight;
    }

    /**
     * Returns whether the elevator has hit the lower limit.
     *
     * @return Whether the elevator has hit the lower limit.
     */
    public boolean hasHitLowerLimit() {
        return wouldHitLowerLimit(getPositionMeters());
    }

    /**
     * Returns whether the elevator has hit the upper limit.
     *
     * @return Whether the elevator has hit the upper limit.
     */
    public boolean hasHitUpperLimit() {
        return wouldHitUpperLimit(getPositionMeters());
    }

    /**
     * Returns the position of the elevator.
     *
     * @return The position of the elevator.
     */
    public double getPositionMeters() {
        return getOutput(0);
    }

    /**
     * Returns the velocity of the elevator.
     *
     * @return The velocity of the elevator.
     */
    public double getVelocityMetersPerSecond() {
        return m_x.get(1, 0);
    }

    /**
     * Returns the elevator current draw.
     *
     * @return The elevator current draw.
     */
    public double getCurrentDrawAmps() {
        // I = V / R - omega / (Kv * R)
        // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
        // spinning 10x faster than the output
        // v = r w, so w = v/r
        double kA = 1 / m_plant.getB().get(1, 0);
        double kV = -m_plant.getA().get(1, 1) * kA;
        double motorVelocityRadPerSec = getVelocityMetersPerSecond() * kV * m_gearbox.KvRadPerSecPerVolt;
        var appliedVoltage = m_u.get(0, 0);
        return m_gearbox.getCurrent(motorVelocityRadPerSec, appliedVoltage)
                * Math.signum(appliedVoltage);
    }

    /**
     * Sets the input voltage for the elevator.
     *
     * @param volts The input voltage.
     */
    public void setInputVoltage(double volts) {
        setInput(volts);
        clampInput(RobotController.getBatteryVoltage());
        double mps = getVelocityMetersPerSecond();
        double staticFriction = -Math.signum(mps) * m_staticFrictionVoltage;
        if (Math.abs(volts) > Math.abs(staticFriction)) {
            m_staticFriction.fill(staticFriction);
            m_usuableInput = m_u.plus(m_staticFriction);
        } else {
            m_usuableInput.fill(0.0);
        }
    }

    /**
     * Updates the state of the elevator.
     *
     * @param currentXhat The current state estimate.
     * @param u           The system inputs (voltage).
     * @param dtSeconds   The time difference between controller updates.
     */
    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        // Calculate updated x-hat from Runge-Kutta.
        var updatedXhat = NumericalIntegration.rkdp(
                (x, _u) -> {
                    Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                    xdot = xdot.plus(VecBuilder.fill(0, -m_effectiveGravity));
                    return xdot;
                },
                currentXhat,
                u,
                dtSeconds);

        // We check for collisions after updating x-hat.
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(m_minHeight, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(m_maxHeight, 0);
        }
        return updatedXhat;
    }
}

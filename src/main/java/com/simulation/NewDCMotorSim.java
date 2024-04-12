// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.simulation;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

/** Represents a simulated DC motor mechanism. */
public class NewDCMotorSim extends NewLinearSystemSim<N2, N1, N2> {
  // Gearbox for the DC motor.
  private final DCMotor m_gearbox;

  // The gearing from the motors to the output.
  private final double m_gearing;

  // The static friction voltage.
  private final double m_staticFrictionVoltage;

  // The static friction matrix
  private final Matrix<N1, N1> m_staticFriction;

  // The usuable input matrix
  private Matrix<N1, N1> m_usuableInput;

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param plant                 The linear system representing the DC motor.
   *                              This system can be created with
   * @param gearbox               The type of and number of motors in the DC motor
   *                              gearbox.
   * @param gearing               The gearing of the DC motor (numbers greater
   *                              than 1 represent reductions).
   * @param staticFrictionVoltage The voltage needed to get the motor turning.
   * @param measurementStdDevs    The standard deviations of the measurements. Can
   *                              be omitted if no
   *                              noise is desired. If present must have 2
   *                              elements. The first element is for position. The
   *                              second element is for velocity.
   */
  public NewDCMotorSim(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      double gearing,
      double staticFrictionVoltage,
      double... measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
    m_staticFrictionVoltage = staticFrictionVoltage;
    m_staticFriction = new Matrix<>(new SimpleMatrix(1, 1));
    m_usuableInput = new Matrix<>(new SimpleMatrix(1, 1));
  }

  /**
   * Creates a simulated DC motor mechanism.
   *
   * @param gearbox               The type of and number of motors in the DC motor
   *                              gearbox.
   * @param gearing               The gearing of the DC motor (numbers greater
   *                              than 1 represent reductions).
   * @param jKgMetersSquared      The moment of inertia of the DC motor. If this
   *                              is unknown, use the
   *                              {@link #DCMotorSim(LinearSystem, DCMotor, double, double...)}
   *                              constructor.
   * @param staticFrictionVoltage The voltage needed to get the motor turning.
   * @param measurementStdDevs    The standard deviations of the measurements. Can
   *                              be omitted if no
   *                              noise is desired. If present must have 2
   *                              elements. The first element is for position. The
   *                              second element is for velocity.
   */
  public NewDCMotorSim(
      DCMotor gearbox,
      double gearing,
      double jKgMetersSquared,
      double staticFrictionVoltage,
      double... measurementStdDevs) {
    super(
        LinearSystemId.createDCMotorSystem(gearbox, jKgMetersSquared, gearing), measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
    m_staticFrictionVoltage = staticFrictionVoltage;
    m_staticFriction = new Matrix<>(new SimpleMatrix(1, 1));
    m_usuableInput = new Matrix<>(new SimpleMatrix(1, 1));
  }

  /**
   * Sets the state of the DC motor.
   *
   * @param angularPositionRad       The new position in radians.
   * @param angularVelocityRadPerSec The new velocity in radians per second.
   */
  public void setState(double angularPositionRad, double angularVelocityRadPerSec) {
    setState(VecBuilder.fill(angularPositionRad, angularVelocityRadPerSec));
  }

  /**
   * Returns the DC motor position.
   *
   * @return The DC motor position.
   */
  public double getAngularPositionRad() {
    return getOutput(0);
  }

  /**
   * Returns the DC motor position in rotations.
   *
   * @return The DC motor position in rotations.
   */
  public double getAngularPositionRotations() {
    return Units.radiansToRotations(getAngularPositionRad());
  }

  /**
   * Returns the DC motor velocity.
   *
   * @return The DC motor velocity.
   */
  public double getAngularVelocityRadPerSec() {
    return getOutput(1);
  }

  /**
   * Returns the DC motor velocity in RPM.
   *
   * @return The DC motor velocity in RPM.
   */
  public double getAngularVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(getAngularVelocityRadPerSec());
  }

  /**
   * Returns the DC motor current draw.
   *
   * @return The DC motor current draw.
   */
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor is
    // spinning
    // 2x faster than the output
    return m_gearbox.getCurrent(getAngularVelocityRadPerSec() * m_gearing, m_u.get(0, 0))
        * Math.signum(m_u.get(0, 0));
  }

  /**
   * Sets the input voltage for the DC motor.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
    clampInput(RobotController.getBatteryVoltage());
    double rpm = getAngularVelocityRPM();
    double staticFriction = -Math.signum(rpm) * m_staticFrictionVoltage;
    if (Math.abs(volts) > Math.abs(staticFriction)) {
      m_staticFriction.fill(staticFriction);
      m_usuableInput = m_u.plus(m_staticFriction);
    }else{
      m_usuableInput.fill(0.0);
    }
  }

  /**
   * Updates the simulation.
   *
   * @param dtSeconds The time between updates.
   */
  @Override
  public void update(double dtSeconds) {
    // Update X. By default, this is the linear system dynamics X = Ax + Bu
    m_x = updateX(m_x, m_usuableInput, dtSeconds);

    // y = cx + du
    m_y = m_plant.calculateY(m_x, m_usuableInput);

    // Add measurement noise.
    if (m_measurementStdDevs != null) {
      m_y = m_y.plus(StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs));
    }
  }
}
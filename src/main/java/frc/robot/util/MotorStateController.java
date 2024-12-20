// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import java.util.function.Supplier;

/**
 * Combines a controller, feedforward, and observer for controlling a mechanism with full state
 * feedback.
 *
 * <p>For everything in this file, "inputs" and "outputs" are defined from the perspective of the
 * plant. This means U is an input and Y is an output (because you give the plant U (powers) and it
 * gives you back a Y (sensor values)). This is the opposite of what they mean from the perspective
 * of the controller (U is an output because that's what goes to the motors and Y is an input
 * because that's what comes back from the sensors).
 *
 * @see {@link edu.wpi.first.math.system.LinearSystemLoop}
 */
public class MotorStateController extends LinearSystemLoop<N2, N1, N2> {
  private Supplier<Matrix<N2, N1>> m_measurement;

  /**
   * Constructs a MotorStateController. By default, the maximum voltage that can be applied is 12.
   *
   * @param plant The state-space plant being controlled.
   * @param q The maximum desired error tolerance for each state.
   * @see {@link edu.wpi.first.math.system.LinearSystem}
   * @see {@link edu.wpi.first.math.controller.LinearQuadraticRegulator}
   * @see {@link edu.wpi.first.math.estimator.KalmanFilter}
   */
  public MotorStateController(
      LinearSystem<N2, N1, N2> plant,
      Vector<N2> q,
      Matrix<N2, N1> stateStdDevs,
      Supplier<Matrix<N2, N1>> measurement) {
    super(
        plant,
        new LinearQuadraticRegulator<>(plant, q, VecBuilder.fill(12.0), 0.02),
        new KalmanFilter<>(
            Nat.N2(), Nat.N2(), plant, stateStdDevs, VecBuilder.fill(0.01, 0.01), 0.02),
        12.0,
        0.02);

    m_measurement = measurement;
  }

  public void setNextR(Matrix<N2, N1> reference) {
    super.setNextR(reference);
  }

  public double calculate() {
    correct(m_measurement.get());
    predict(0.02);
    return getU(0);
  }

  public double calculate(Matrix<N2, N1> reference) {
    this.setNextR(reference);
    return this.calculate();
  }
}

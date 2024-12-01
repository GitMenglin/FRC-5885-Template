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
import frc.robot.Constants.StateModelConstants;

/** Add your docs here. */
public class MotorStateController extends LinearSystemLoop<N2, N1, N2> {
  public MotorStateController(LinearSystem<N2, N1, N2> plant, Vector<N2> q) {
    super(
        plant,
        new LinearQuadraticRegulator<>(plant, q, VecBuilder.fill(12.0), 0.02),
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            plant,
            StateModelConstants.kStateStdDevs,
            StateModelConstants.kMeasurementStdDevs,
            0.02),
        12.0,
        0.02);
  }

  public void setNextR(Matrix<N2, N1> reference) {
    super.setNextR(reference);
  }

  public double calculate(Matrix<N2, N1> measurement) {
    correct(measurement);
    predict(0.02);
    return getU(0);
  }

  public double calculate(Matrix<N2, N1> measurement, Matrix<N2, N1> reference) {
    this.setNextR(reference);
    return calculate(measurement);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleLocation;
import frc.robot.Constants.StateModelConstants;
import frc.robot.util.MotorStateController;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final double m_wheelRadius = DriveConstants.kWheelDiameterMeters / 2.0;

  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_index;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private final PIDController m_driveFeedback;
  private final PIDController m_turnFeedback;
  private Rotation2d m_angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double m_speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d m_turnRelativeOffset = null; // Relative + Offset = Absolute

  private final MotorStateController m_driveState =
      new MotorStateController(
          LinearSystemId.createDCMotorSystem(
              StateModelConstants.kVdrive, StateModelConstants.kAdrive),
          StateModelConstants.kQdrive);

  private final MotorStateController m_turnState =
      new MotorStateController(
          LinearSystemId.createDCMotorSystem(
              StateModelConstants.kVturn, StateModelConstants.kAturn),
          StateModelConstants.kQturn);

  private static boolean m_isStateSpace = false;

  public Module(ModuleIO io, ModuleLocation location) {
    m_io = io;
    m_index = location.ordinal();

    SmartDashboard.putBoolean(getClass().getSimpleName() + "/isStateSpace", m_isStateSpace);

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.kCurrentMode) {
      case REAL:
        m_driveFeedforward =
            new SimpleMotorFeedforward(DriveConstants.kSdrive, DriveConstants.kVdrive);
        m_driveFeedback =
            new PIDController(
                DriveConstants.kPdrive, DriveConstants.kIdrive, DriveConstants.kDdrive);
        m_turnFeedback =
            new PIDController(DriveConstants.kPturn, DriveConstants.kIturn, DriveConstants.kDturn);
        break;
      case REPLAY:
        m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        m_driveFeedback = new PIDController(0.05, 0.0, 0.0);
        m_turnFeedback = new PIDController(7.0, 0.0, 0.0);
        break;
      case SIM:
        m_driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
        m_driveFeedback = new PIDController(0.1, 0.0, 0.0);
        m_turnFeedback = new PIDController(10.0, 0.0, 0.0);
        break;
      default:
        m_driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
        m_driveFeedback = new PIDController(0.0, 0.0, 0.0);
        m_turnFeedback = new PIDController(0.0, 0.0, 0.0);
        break;
    }

    m_turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(m_index), m_inputs);
    m_isStateSpace =
        SmartDashboard.getBoolean(getClass().getSimpleName() + "/isStateSpace", m_isStateSpace);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (m_turnRelativeOffset == null && m_inputs.turnAbsolutePosition.getRadians() != 0.0) {
      m_turnRelativeOffset = m_inputs.turnAbsolutePosition.minus(m_inputs.turnPosition);
    }

    // Run closed loop turn control
    if (m_angleSetpoint != null) {
      if (!m_isStateSpace) {
        m_io.setTurnVoltage(
            m_turnFeedback.calculate(getAngle().getRadians(), m_angleSetpoint.getRadians()));
      } else {
        Matrix<N2, N1> turnReference =
            MatBuilder.fill(
                Nat.N2(),
                Nat.N1(),
                m_angleSetpoint.getRadians(),
                (m_angleSetpoint.getRadians() - getAngle().getRadians()) / 0.02);
        Matrix<N2, N1> turnMeasurement =
            MatBuilder.fill(
                Nat.N2(), Nat.N1(), getAngle().getRadians(), m_inputs.turnVelocityRadPerSec);
        m_io.setTurnVoltage(m_turnState.calculate(turnMeasurement, turnReference));
      }

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (m_speedSetpoint != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint = m_speedSetpoint * Math.cos(m_turnFeedback.getPositionError());

        double velocityRadPerSec = adjustSpeedSetpoint / m_wheelRadius;

        if (!m_isStateSpace) {
          // Run drive controller
          m_io.setDriveVoltage(
              m_driveFeedforward.calculate(velocityRadPerSec)
                  + m_driveFeedback.calculate(m_inputs.driveVelocityRadPerSec, velocityRadPerSec));
        } else {
          Matrix<N2, N1> driveReference =
              MatBuilder.fill(
                  Nat.N2(),
                  Nat.N1(),
                  getPositionMeters() + adjustSpeedSetpoint * 0.02,
                  adjustSpeedSetpoint);
          Matrix<N2, N1> driveMeasurement =
              MatBuilder.fill(Nat.N2(), Nat.N1(), getPositionMeters(), getVelocityMetersPerSec());
          m_io.setDriveVoltage(m_driveState.calculate(driveMeasurement, driveReference));
        }
      }
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    m_angleSetpoint = optimizedState.angle;
    m_speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    m_angleSetpoint = new Rotation2d();

    // Open loop drive control
    m_io.setDriveVoltage(volts);
    m_speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    m_io.setTurnVoltage(0.0);
    m_io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    m_angleSetpoint = null;
    m_speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    m_io.setDriveBrakeMode(enabled);
    m_io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (m_turnRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return m_inputs.turnPosition.plus(m_turnRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return m_inputs.drivePositionRad * m_wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return m_inputs.driveVelocityRadPerSec * m_wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return m_inputs.driveVelocityRadPerSec;
  }
}

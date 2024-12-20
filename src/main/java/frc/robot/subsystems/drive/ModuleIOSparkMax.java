// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ModuleLocation;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax m_driveSparkMax;
  private final CANSparkMax m_turnSparkMax;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnRelativeEncoder;
  private final AnalogInput m_turnAbsoluteEncoder;

  private final boolean m_isTurnMotorInverted = true;
  private final Rotation2d m_absoluteEncoderOffset;

  public ModuleIOSparkMax(ModuleLocation location) {
    switch (location) {
      case FRONT_LEFT:
        m_driveSparkMax =
            new CANSparkMax(SwerveModuleConstants.kFrontLeftDriveMotorPort, MotorType.kBrushless);
        m_turnSparkMax =
            new CANSparkMax(SwerveModuleConstants.kFrontLeftTurningMotorPort, MotorType.kBrushless);
        m_turnAbsoluteEncoder = new AnalogInput(SwerveModuleConstants.kFrontLeftEncoderPort);
        m_absoluteEncoderOffset = new Rotation2d(SwerveModuleConstants.kFrontLeftEncoderOffset);
        break;
      case FRONT_RIGHT:
        m_driveSparkMax =
            new CANSparkMax(SwerveModuleConstants.kFrontRightDriveMotorPort, MotorType.kBrushless);
        m_turnSparkMax =
            new CANSparkMax(
                SwerveModuleConstants.kFrontRightTurningMotorPort, MotorType.kBrushless);
        m_turnAbsoluteEncoder = new AnalogInput(SwerveModuleConstants.kFrontRightEncoderPort);
        m_absoluteEncoderOffset = new Rotation2d(SwerveModuleConstants.kFrontRightEncoderOffset);
        break;
      case BACK_LEFT:
        m_driveSparkMax =
            new CANSparkMax(SwerveModuleConstants.kBackLeftDriveMotorPort, MotorType.kBrushless);
        m_turnSparkMax =
            new CANSparkMax(SwerveModuleConstants.kBackLeftTurningMotorPort, MotorType.kBrushless);
        m_turnAbsoluteEncoder = new AnalogInput(SwerveModuleConstants.kBackLeftEncoderPort);
        m_absoluteEncoderOffset = new Rotation2d(SwerveModuleConstants.kBackLeftEncoderOffset);
        break;
      case BACK_RIGHT:
        m_driveSparkMax =
            new CANSparkMax(SwerveModuleConstants.kBackRightDriveMotorPort, MotorType.kBrushless);
        m_turnSparkMax =
            new CANSparkMax(SwerveModuleConstants.kBackRightTurningMotorPort, MotorType.kBrushless);
        m_turnAbsoluteEncoder = new AnalogInput(SwerveModuleConstants.kBackRightEncoderPort);
        m_absoluteEncoderOffset = new Rotation2d(SwerveModuleConstants.kBackRightEncoderOffset);
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // m_driveSparkMax.restoreFactoryDefaults();
    // m_turnSparkMax.restoreFactoryDefaults();

    m_driveSparkMax.setCANTimeout(250);
    m_turnSparkMax.setCANTimeout(250);

    m_driveEncoder = m_driveSparkMax.getEncoder();
    m_turnRelativeEncoder = m_turnSparkMax.getEncoder();

    m_turnSparkMax.setInverted(m_isTurnMotorInverted);
    m_driveSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kDriveCurrentLimit);
    m_turnSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kTurnCurrentLimit);
    m_driveSparkMax.enableVoltageCompensation(12.0);
    m_turnSparkMax.enableVoltageCompensation(12.0);

    m_driveEncoder.setPosition(0.0);
    m_driveEncoder.setMeasurementPeriod(10);
    m_driveEncoder.setAverageDepth(2);

    m_turnRelativeEncoder.setPosition(0.0);
    m_turnRelativeEncoder.setMeasurementPeriod(10);
    m_turnRelativeEncoder.setAverageDepth(2);

    m_driveSparkMax.setCANTimeout(0);
    m_turnSparkMax.setCANTimeout(0);

    // m_driveSparkMax.burnFlash();
    // m_turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(m_driveEncoder.getPosition())
            / SwerveModuleConstants.kDriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_driveEncoder.getVelocity())
            / SwerveModuleConstants.kDriveGearRatio;
    inputs.driveAppliedVolts = m_driveSparkMax.getAppliedOutput() * m_driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {m_driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        new Rotation2d(
                (m_turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V())
                    * 2.0
                    * Math.PI)
            .minus(m_absoluteEncoderOffset);
    inputs.turnPosition = inputs.turnAbsolutePosition;
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_turnRelativeEncoder.getVelocity())
            / SwerveModuleConstants.kTurnGearRatio;
    inputs.turnAppliedVolts = m_turnSparkMax.getAppliedOutput() * m_turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {m_turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    m_driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    m_turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDriveSubsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModuleNEO implements SwerveModuleIO {

  private SparkMax m_driveMotor;
  private SparkMax m_turnMotor;

  private AnalogInput m_turnAbsoluteEncoder;
  private Rotation2d m_turnAbsoluteEncoderOffset;

  private final RelativeEncoder m_driveDefaultEncoder;
  private final RelativeEncoder m_turnRelativeEncoder;

  public SwerveModuleNEO(
      int driveMotorId,
      int turnMotorId,
      int turnAbsoluteEncoderId,
      Rotation2d turnAbsoluteEncoderOffset,
      boolean turnMotorReversed,
      boolean driveMotorReversed) {
    m_driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);
    m_turnAbsoluteEncoder = new AnalogInput(turnAbsoluteEncoderId);
    m_turnAbsoluteEncoderOffset = turnAbsoluteEncoderOffset;

    m_driveDefaultEncoder = m_driveMotor.getEncoder();
    m_turnRelativeEncoder = m_turnMotor.getEncoder();

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turnConfig = new SparkMaxConfig();

    driveConfig
        .inverted(driveMotorReversed)
        .idleMode(IdleMode.kBrake);
    turnConfig
        .inverted(turnMotorReversed)
        .idleMode(IdleMode.kBrake);

    driveConfig.encoder
        .positionConversionFactor(SwerveConstants.ModuleConstants.kDriveEncoderRot2Meter)
        .velocityConversionFactor(SwerveConstants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turnConfig.encoder
        .positionConversionFactor(SwerveConstants.ModuleConstants.kTurningEncoderRot2Rad)
        .velocityConversionFactor(SwerveConstants.ModuleConstants.kTurningEncoderRPM2RadPerSec);

    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Optimize the spark max frame timings to clean up the CAN bus
    // SparkMaxConfigurer.setFrameTimingsOptmized(m_driveMotor);
    // SparkMaxConfigurer.setFrameTimingsOptmized(m_turnMotor);

    m_turnRelativeEncoder.setPosition(getAbsoluteEncoderValue().getRadians());
  }

  public void updateInputs(SwerveModuleIOInputs inputs) {

    inputs.drivePositionMeters = m_driveDefaultEncoder.getPosition();
    inputs.driveVelocityMetersPerSec = m_driveDefaultEncoder.getVelocity();

    inputs.driveTemperatureCelsius = m_driveMotor.getMotorTemperature();
    inputs.driveCurrent = m_driveMotor.getOutputCurrent();
    inputs.driveVoltage = m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();

    inputs.turnAbsolutePositionRad = getAbsoluteEncoderValue().getRadians();

    // Use the absolute encoder value if UseExternalEncoders is true
    if (SwerveConstants.kUseExternalEncoders) {
      inputs.turnPositionRad = inputs.turnAbsolutePositionRad;
    } else {
      inputs.turnPositionRad = m_turnRelativeEncoder.getPosition();
    }

    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_turnRelativeEncoder.getVelocity())
        / (1 / SwerveConstants.ModuleConstants.kTurningMotorGearRatio);

    inputs.turnTemperature = m_turnMotor.getMotorTemperature();
    inputs.turnCurrent = m_turnMotor.getOutputCurrent();
    inputs.turnVoltage = m_turnMotor.getAppliedOutput() * m_turnMotor.getBusVoltage();
  }

  ////////////////////////////////////////
  // Calculate angle from absolute encoder
  public Rotation2d getAbsoluteEncoderValue() {
    double absolutePositionPercent = (m_turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V());
    return new Rotation2d(absolutePositionPercent * 2.0 * Math.PI)
        .minus(m_turnAbsoluteEncoderOffset);
  }

  public void setDriveVoltage(double voltage) {
    m_driveMotor.setVoltage(voltage);
  }

  public void setTurnVoltage(double voltage) {
    m_turnMotor.setVoltage(voltage);
  }

  public void setDriveBrakeMode(boolean enable) {
    // m_driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    throw new UnsupportedOperationException("setDriveBrakeMode not implemented");
  }

  public void setTurnBrakeMode(boolean enable) {
    // m_turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    throw new UnsupportedOperationException("setTurnBrakeMode not implemented");
  }
}

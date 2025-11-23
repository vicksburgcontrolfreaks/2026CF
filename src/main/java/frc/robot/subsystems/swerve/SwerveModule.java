// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_steerMotor;

  private final RelativeEncoder m_driveEncoder;
  private final SparkAbsoluteEncoder m_steerEncoder;

  private final SparkClosedLoopController m_drivePIDController;
  private final SparkClosedLoopController m_steerPIDController;

  private final double m_offset;
  private final String m_moduleName;

  public SwerveModule(
      int driveMotorId,
      int steerMotorId,
      double offset,
      String moduleName) {
    m_offset = offset;
    m_moduleName = moduleName;

    // Initialize drive motor
    m_driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_drivePIDController = m_driveMotor.getClosedLoopController();

    // Initialize steer motor
    m_steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);
    m_steerEncoder = m_steerMotor.getAbsoluteEncoder();
    m_steerPIDController = m_steerMotor.getClosedLoopController();

    configureDriveMotor();
    configureSteerMotor();
  }

  private void configureDriveMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    // Factory reset and restore defaults
    config.inverted(SwerveConstants.kDriveMotorInverted);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(SwerveConstants.kDriveMotorCurrentLimit);

    // Configure encoder
    config.encoder
      .positionConversionFactor(SwerveConstants.kWheelCircumferenceMeters / SwerveConstants.kDriveGearRatio)
      .velocityConversionFactor(SwerveConstants.kWheelCircumferenceMeters / SwerveConstants.kDriveGearRatio / 60.0);

    // Configure PID
    config.closedLoop
      .pid(SwerveConstants.kDriveP, SwerveConstants.kDriveI, SwerveConstants.kDriveD)
      .velocityFF(SwerveConstants.kDriveFF)
      .outputRange(-1, 1);

    m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_driveEncoder.setPosition(0);
  }

  private void configureSteerMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(SwerveConstants.kSteerMotorInverted);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(SwerveConstants.kSteerMotorCurrentLimit);

    // Configure absolute encoder
    config.absoluteEncoder
      .inverted(SwerveConstants.kSteerEncoderInverted)
      .positionConversionFactor(2 * Math.PI)
      .velocityConversionFactor(2 * Math.PI / 60.0);

    // Configure PID - feedback device is absolute encoder
    config.closedLoop
      .pid(SwerveConstants.kSteerP, SwerveConstants.kSteerI, SwerveConstants.kSteerD)
      .feedbackSensor(com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
      .positionWrappingEnabled(true)
      .positionWrappingMinInput(0)
      .positionWrappingMaxInput(2 * Math.PI)
      .outputRange(-1, 1);

    m_steerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      m_driveEncoder.getVelocity(),
      new Rotation2d(getSteerAngle())
    );
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_driveEncoder.getPosition(),
      new Rotation2d(getSteerAngle())
    );
  }

  private double getSteerAngle() {
    double angle = m_steerEncoder.getPosition() - (m_offset * 2 * Math.PI);
    return angle;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle;

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
      correctedDesiredState,
      new Rotation2d(getSteerAngle())
    );

    // Set drive motor velocity
    m_drivePIDController.setReference(
      optimizedDesiredState.speedMetersPerSecond,
      SparkMax.ControlType.kVelocity
    );

    // Set steer motor position
    double steerSetpoint = optimizedDesiredState.angle.getRadians() + (m_offset * 2 * Math.PI);
    m_steerPIDController.setReference(
      steerSetpoint,
      SparkMax.ControlType.kPosition
    );
  }

  public void stop() {
    m_driveMotor.set(0);
    m_steerMotor.set(0);
  }

  public String getModuleName() {
    return m_moduleName;
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public double getSteerPosition() {
    return getSteerAngle();
  }

  public double getAbsoluteEncoderRaw() {
    return m_steerEncoder.getPosition();
  }

  public void resetDriveEncoder() {
    m_driveEncoder.setPosition(0);
  }
}

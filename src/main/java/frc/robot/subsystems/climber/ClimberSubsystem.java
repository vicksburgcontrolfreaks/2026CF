// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/// Motor Inversion: The right motor is set to follow the left 
/// motor without inversion. If your mechanism requires the motors 
/// to spin in opposite directions, change line 29 in ClimberSubsystem.java from false to true.

/// Position Tuning: The default extended position is set
///  to 100 rotations. You'll need to test and adjust this 
/// value in Constants.java:155 based on your actual climber mechanism.

/// PID Tuning: The position PID is set with P=0.1. You may need to 
/// tune this in Constants.java:149 for smooth, responsive movement.

/// Initial Position: The encoders are reset to 0 on startup, 
/// assuming the climber starts fully retracted. If this isn't 
/// the case, you'll need to manually reset using resetEncoders().

package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.TelemetryConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax m_leftClimberMotor;
  private final SparkMax m_rightClimberMotor;

  private double m_targetPosition = ClimberConstants.kRetractedPosition;
  private int m_telemetryCounter = 0;

  private final NetworkTable m_telemetryTable;
  private final DoublePublisher m_leftPositionPub;
  private final DoublePublisher m_rightPositionPub;
  private final DoublePublisher m_targetPositionPub;
  private final DoublePublisher m_leftSpeedPub;
  private final DoublePublisher m_rightSpeedPub;
  private final DoublePublisher m_leftCurrentPub;
  private final DoublePublisher m_rightCurrentPub;

  public ClimberSubsystem() {
    m_leftClimberMotor = new SparkMax(ClimberConstants.kLeftClimberMotorId, MotorType.kBrushless);
    m_rightClimberMotor = new SparkMax(ClimberConstants.kRightClimberMotorId, MotorType.kBrushless);

    m_leftClimberMotor.configure(ClimberConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure right motor to follow left motor (not inverted)
    // TODO: Test and adjust inversion if motors need to run in opposite directions
    var rightConfig = new com.revrobotics.spark.config.SparkMaxConfig();
    rightConfig.follow(ClimberConstants.kLeftClimberMotorId, false);
    rightConfig.apply(ClimberConstants.config); // Apply base config settings
    m_rightClimberMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Reset encoders to zero on startup (assumes robot starts retracted)
    m_leftClimberMotor.getEncoder().setPosition(0.0);
    m_rightClimberMotor.getEncoder().setPosition(0.0);

    m_telemetryTable = NetworkTableInstance.getDefault().getTable("Climber");
    m_leftPositionPub = m_telemetryTable.getDoubleTopic("Left Position").publish();
    m_rightPositionPub = m_telemetryTable.getDoubleTopic("Right Position").publish();
    m_targetPositionPub = m_telemetryTable.getDoubleTopic("Target Position").publish();
    m_leftSpeedPub = m_telemetryTable.getDoubleTopic("Left Speed").publish();
    m_rightSpeedPub = m_telemetryTable.getDoubleTopic("Right Speed").publish();
    m_leftCurrentPub = m_telemetryTable.getDoubleTopic("Left Current").publish();
    m_rightCurrentPub = m_telemetryTable.getDoubleTopic("Right Current").publish();
  }

  /**
   * Extend the climber to the fully extended position
   */
  public void extend() {
    setPosition(ClimberConstants.kExtendedPosition);
  }

  /**
   * Retract the climber to the fully retracted position
   */
  public void retract() {
    setPosition(ClimberConstants.kRetractedPosition);
  }

  /**
   * Set the climber to a specific position
   * @param position Target position in rotations
   */
  public void setPosition(double position) {
    // Clamp position to soft limits
    m_targetPosition = Math.max(ClimberConstants.kSoftLimitMin,
                                Math.min(ClimberConstants.kSoftLimitMax, position));
  }

  /**
   * Get the current position of the left climber
   * @return Current position in rotations
   */
  public double getLeftPosition() {
    return m_leftClimberMotor.getEncoder().getPosition();
  }

  /**
   * Get the current position of the right climber
   * @return Current position in rotations
   */
  public double getRightPosition() {
    return m_rightClimberMotor.getEncoder().getPosition();
  }

  /**
   * Get the average position of both climbers
   * @return Average position in rotations
   */
  public double getPosition() {
    return (getLeftPosition() + getRightPosition()) / 2.0;
  }

  /**
   * Get the target position of the climber
   * @return Target position in rotations
   */
  public double getTargetPosition() {
    return m_targetPosition;
  }

  /**
   * Check if the climber is at the target position
   * @return True if within tolerance of target position
   */
  public boolean isAtTarget() {
    return Math.abs(getPosition() - m_targetPosition) < ClimberConstants.kPositionTolerance;
  }

  /**
   * Check if the climber is fully extended
   * @return True if at extended position
   */
  public boolean isExtended() {
    return Math.abs(getPosition() - ClimberConstants.kExtendedPosition) < ClimberConstants.kPositionTolerance;
  }

  /**
   * Check if the climber is fully retracted
   * @return True if at retracted position
   */
  public boolean isRetracted() {
    return Math.abs(getPosition() - ClimberConstants.kRetractedPosition) < ClimberConstants.kPositionTolerance;
  }

  /**
   * Manually control the climber with a speed value
   * @param speed Speed from -1.0 to 1.0 (positive extends, negative retracts)
   */
  public void setSpeed(double speed) {
    // Clamp speed
    speed = Math.max(-1.0, Math.min(1.0, speed));

    // Check soft limits when moving manually
    double currentPosition = getPosition();
    if (speed > 0 && currentPosition >= ClimberConstants.kSoftLimitMax) {
      m_leftClimberMotor.set(0); // Stop if at max limit
    } else if (speed < 0 && currentPosition <= ClimberConstants.kSoftLimitMin) {
      m_leftClimberMotor.set(0); // Stop if at min limit
    } else {
      m_leftClimberMotor.set(speed);
      // Right motor follows left automatically
    }
  }

  /**
   * Stop both climber motors
   */
  public void stop() {
    m_leftClimberMotor.set(0);
    // Right motor will stop automatically since it follows left
  }

  /**
   * Reset both encoders to zero (call this when climber is fully retracted)
   */
  public void resetEncoders() {
    m_leftClimberMotor.getEncoder().setPosition(0.0);
    m_rightClimberMotor.getEncoder().setPosition(0.0);
    m_targetPosition = 0.0;
  }

  @Override
  public void periodic() {
    // Use position control to move to target position (left motor is leader)
    m_leftClimberMotor.getClosedLoopController().setSetpoint(
      m_targetPosition,
      ControlType.kPosition
    );

    // Update telemetry
    m_telemetryCounter++;
    if (m_telemetryCounter >= TelemetryConstants.kTelemetryUpdatePeriod) {
      m_telemetryCounter = 0;

      m_leftPositionPub.set(getLeftPosition());
      m_rightPositionPub.set(getRightPosition());
      m_targetPositionPub.set(m_targetPosition);
      m_leftSpeedPub.set(m_leftClimberMotor.get());
      m_rightSpeedPub.set(m_rightClimberMotor.get());
      m_leftCurrentPub.set(m_leftClimberMotor.getOutputCurrent());
      m_rightCurrentPub.set(m_rightClimberMotor.getOutputCurrent());
    }
  }
}

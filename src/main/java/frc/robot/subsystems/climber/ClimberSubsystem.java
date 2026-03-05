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
  private final SparkMax m_ClimberMotor;

  private double m_targetPosition = ClimberConstants.kRetractedPosition;
  private int m_telemetryCounter = 0;

  private final NetworkTable m_telemetryTable;
  private final DoublePublisher m_leftPositionPub;
  private final DoublePublisher m_encoderTicksPub;
  private final DoublePublisher m_targetPositionPub;
  private final DoublePublisher m_leftSpeedPub;
  private final DoublePublisher m_leftCurrentPub;

  public ClimberSubsystem() {
    m_ClimberMotor = new SparkMax(ClimberConstants.kClimberMotorId, MotorType.kBrushless);

    m_ClimberMotor.configure(ClimberConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Reset encoders to zero on startup (assumes robot starts retracted)
    m_ClimberMotor.getEncoder().setPosition(0.0);

    m_telemetryTable = NetworkTableInstance.getDefault().getTable("Climber");
    m_leftPositionPub = m_telemetryTable.getDoubleTopic("Left Position").publish();
    m_encoderTicksPub = m_telemetryTable.getDoubleTopic("Encoder Ticks").publish();
    m_targetPositionPub = m_telemetryTable.getDoubleTopic("Target Position").publish();
    m_leftSpeedPub = m_telemetryTable.getDoubleTopic("Left Speed").publish();
    m_leftCurrentPub = m_telemetryTable.getDoubleTopic("Left Current").publish();
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
   * Get the current position of the climber
   * @return Current position in rotations
   */
  public double getPosition() {
    return m_ClimberMotor.getEncoder().getPosition();
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
      m_ClimberMotor.set(0); // Stop if at max limit
    } else if (speed < 0 && currentPosition <= ClimberConstants.kSoftLimitMin) {
      m_ClimberMotor.set(0); // Stop if at min limit
    } else {
      m_ClimberMotor.set(speed);
    }
  }

  /**
   * Stop the climber motor
   */
  public void stop() {
    m_ClimberMotor.set(0);
  }

  /**
   * Reset encoder to zero (call this when climber is fully retracted)
   */
  public void resetEncoders() {
    m_ClimberMotor.getEncoder().setPosition(0.0);
    m_targetPosition = 0.0;
  }

  @Override
  public void periodic() {
    // Use position control to move to target position
    m_ClimberMotor.getClosedLoopController().setSetpoint(
      m_targetPosition,
      ControlType.kPosition
    );

    // Update telemetry
    m_telemetryCounter++;
    if (m_telemetryCounter >= TelemetryConstants.kTelemetryUpdatePeriod) {
      m_telemetryCounter = 0;

      m_leftPositionPub.set(getPosition());
      m_encoderTicksPub.set(m_ClimberMotor.getEncoder().getPosition());
      m_targetPositionPub.set(m_targetPosition);
      m_leftSpeedPub.set(m_ClimberMotor.get());
      m_leftCurrentPub.set(m_ClimberMotor.getOutputCurrent());
    }
  }
}

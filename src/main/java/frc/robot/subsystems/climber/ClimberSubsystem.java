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
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ClimberConfig;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.TelemetryConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax m_ClimberMotor;

  private double m_targetPosition = ClimberConstants.kRetractedPosition;
  private int m_telemetryCounter = 0;

  private final NetworkTable m_telemetryTable;
  private final DoublePublisher m_leftPositionPub;
  private final DoublePublisher m_targetPositionPub;
  private final DoublePublisher m_leftSpeedPub;
  private final DoublePublisher m_leftCurrentPub;

  // Configurable constants (via NetworkTables)
  private int m_motorCurrentLimit = ClimberConstants.kMotorCurrentLimit;
  private double m_positionP = ClimberConstants.kPositionP;
  private double m_positionI = ClimberConstants.kPositionI;
  private double m_positionD = ClimberConstants.kPositionD;
  private double m_retractedPosition = ClimberConstants.kRetractedPosition;
  private double m_extendedPosition = ClimberConstants.kExtendedPosition;
  private double m_positionTolerance = ClimberConstants.kPositionTolerance;
  private double m_manualSpeed = ClimberConstants.kManualSpeed;
  private double m_softLimitMin = ClimberConstants.kSoftLimitMin;
  private double m_softLimitMax = ClimberConstants.kSoftLimitMax;
  private int m_telemetryUpdatePeriod = TelemetryConstants.kTelemetryUpdatePeriod;

  // NetworkTables subscribers for configurable constants
  private final DoubleSubscriber m_motorCurrentLimitSub;
  private final DoubleSubscriber m_positionPSub;
  private final DoubleSubscriber m_positionISub;
  private final DoubleSubscriber m_positionDSub;
  private final DoubleSubscriber m_retractedPositionSub;
  private final DoubleSubscriber m_extendedPositionSub;
  private final DoubleSubscriber m_positionToleranceSub;
  private final DoubleSubscriber m_manualSpeedSub;
  private final DoubleSubscriber m_softLimitMinSub;
  private final DoubleSubscriber m_softLimitMaxSub;
  private final DoubleSubscriber m_telemetryUpdatePeriodSub;

  public ClimberSubsystem() {
    m_ClimberMotor = new SparkMax(ClimberConstants.kClimberMotorId, MotorType.kBrushless);

    m_ClimberMotor.configure(ClimberConfig.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Reset encoders to zero on startup (assumes robot starts retracted)
    m_ClimberMotor.getEncoder().setPosition(0.0);

    m_telemetryTable = NetworkTableInstance.getDefault().getTable("Climber");
    m_leftPositionPub = m_telemetryTable.getDoubleTopic("Left Position").publish();
    m_targetPositionPub = m_telemetryTable.getDoubleTopic("Target Position").publish();
    m_leftSpeedPub = m_telemetryTable.getDoubleTopic("Left Speed").publish();
    m_leftCurrentPub = m_telemetryTable.getDoubleTopic("Left Current").publish();

    // Initialize NetworkTables subscribers for configurable constants
    NetworkTable configTable = NetworkTableInstance.getDefault().getTable("Climber/Config");
    m_motorCurrentLimitSub = configTable.getDoubleTopic("Motor Current Limit").subscribe(m_motorCurrentLimit);
    m_positionPSub = configTable.getDoubleTopic("Position P").subscribe(m_positionP);
    m_positionISub = configTable.getDoubleTopic("Position I").subscribe(m_positionI);
    m_positionDSub = configTable.getDoubleTopic("Position D").subscribe(m_positionD);
    m_retractedPositionSub = configTable.getDoubleTopic("Retracted Position").subscribe(m_retractedPosition);
    m_extendedPositionSub = configTable.getDoubleTopic("Extended Position").subscribe(m_extendedPosition);
    m_positionToleranceSub = configTable.getDoubleTopic("Position Tolerance").subscribe(m_positionTolerance);
    m_manualSpeedSub = configTable.getDoubleTopic("Manual Speed").subscribe(m_manualSpeed);
    m_softLimitMinSub = configTable.getDoubleTopic("Soft Limit Min").subscribe(m_softLimitMin);
    m_softLimitMaxSub = configTable.getDoubleTopic("Soft Limit Max").subscribe(m_softLimitMax);
    m_telemetryUpdatePeriodSub = configTable.getDoubleTopic("Telemetry Update Period").subscribe(m_telemetryUpdatePeriod);
  }

  /**
   * Extend the climber to the fully extended position
   */
  public void extend() {
    setPosition(getExtendedPosition());
  }

  /**
   * Retract the climber to the fully retracted position
   */
  public void retract() {
    setPosition(getRetractedPosition());
  }

  /**
   * Set the climber to a specific position
   * @param position Target position in rotations
   */
  public void setPosition(double position) {
    // Clamp position to soft limits
    m_targetPosition = Math.max(getSoftLimitMin(),
                                Math.min(getSoftLimitMax(), position));
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
    return Math.abs(getPosition() - m_targetPosition) < getPositionTolerance();
  }

  /**
   * Check if the climber is fully extended
   * @return True if at extended position
   */
  public boolean isExtended() {
    return Math.abs(getPosition() - getExtendedPosition()) < getPositionTolerance();
  }

  /**
   * Check if the climber is fully retracted
   * @return True if at retracted position
   */
  public boolean isRetracted() {
    return Math.abs(getPosition() - getRetractedPosition()) < getPositionTolerance();
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
    if (speed > 0 && currentPosition >= getSoftLimitMax()) {
      m_ClimberMotor.set(0); // Stop if at max limit
    } else if (speed < 0 && currentPosition <= getSoftLimitMin()) {
      m_ClimberMotor.set(0); // Stop if at min limit
    } else {
      m_ClimberMotor.set(speed);
    }
  }

  // Getters and setters for configurable constants
  public int getMotorCurrentLimit() {
    return m_motorCurrentLimit;
  }

  public void setMotorCurrentLimit(int limit) {
    m_motorCurrentLimit = limit;
  }

  public double getPositionP() {
    return m_positionP;
  }

  public void setPositionP(double p) {
    m_positionP = p;
    updatePositionPID();
  }

  public double getPositionI() {
    return m_positionI;
  }

  public void setPositionI(double i) {
    m_positionI = i;
    updatePositionPID();
  }

  public double getPositionD() {
    return m_positionD;
  }

  public void setPositionD(double d) {
    m_positionD = d;
    updatePositionPID();
  }

  public double getRetractedPosition() {
    return m_retractedPosition;
  }

  public void setRetractedPosition(double position) {
    m_retractedPosition = position;
  }

  public double getExtendedPosition() {
    return m_extendedPosition;
  }

  public void setExtendedPosition(double position) {
    m_extendedPosition = position;
  }

  public double getPositionTolerance() {
    return m_positionTolerance;
  }

  public void setPositionTolerance(double tolerance) {
    m_positionTolerance = tolerance;
  }

  public double getManualSpeed() {
    return m_manualSpeed;
  }

  public void setManualSpeed(double speed) {
    m_manualSpeed = speed;
  }

  public double getSoftLimitMin() {
    return m_softLimitMin;
  }

  public void setSoftLimitMin(double limit) {
    m_softLimitMin = limit;
  }

  public double getSoftLimitMax() {
    return m_softLimitMax;
  }

  public void setSoftLimitMax(double limit) {
    m_softLimitMax = limit;
  }

  public int getTelemetryUpdatePeriod() {
    return m_telemetryUpdatePeriod;
  }

  public void setTelemetryUpdatePeriod(int period) {
    m_telemetryUpdatePeriod = period;
  }

  /**
   * Update climber motor PID values by reconfiguring motor
   */
  private void updatePositionPID() {
    com.revrobotics.spark.config.SparkMaxConfig config = new com.revrobotics.spark.config.SparkMaxConfig();
    config.closedLoop
      .pid(m_positionP, m_positionI, m_positionD);

    m_ClimberMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
    // Read configurable values from NetworkTables
    double newMotorCurrentLimit = m_motorCurrentLimitSub.get();
    if (newMotorCurrentLimit != m_motorCurrentLimit) {
      m_motorCurrentLimit = (int) newMotorCurrentLimit;
    }

    double newPositionP = m_positionPSub.get();
    double newPositionI = m_positionISub.get();
    double newPositionD = m_positionDSub.get();
    if (newPositionP != m_positionP || newPositionI != m_positionI || newPositionD != m_positionD) {
      m_positionP = newPositionP;
      m_positionI = newPositionI;
      m_positionD = newPositionD;
      updatePositionPID();
    }

    m_retractedPosition = m_retractedPositionSub.get();
    m_extendedPosition = m_extendedPositionSub.get();
    m_positionTolerance = m_positionToleranceSub.get();
    m_manualSpeed = m_manualSpeedSub.get();
    m_softLimitMin = m_softLimitMinSub.get();
    m_softLimitMax = m_softLimitMaxSub.get();
    m_telemetryUpdatePeriod = (int) m_telemetryUpdatePeriodSub.get();

    // Use position control to move to target position
    m_ClimberMotor.getClosedLoopController().setSetpoint(
      m_targetPosition,
      ControlType.kPosition
    );

    // Update telemetry
    m_telemetryCounter++;
    if (m_telemetryCounter >= getTelemetryUpdatePeriod()) {
      m_telemetryCounter = 0;

      m_leftPositionPub.set(getPosition());
      m_targetPositionPub.set(m_targetPosition);
      m_leftSpeedPub.set(m_ClimberMotor.get());
      m_leftCurrentPub.set(m_ClimberMotor.getOutputCurrent());
    }
  }
}

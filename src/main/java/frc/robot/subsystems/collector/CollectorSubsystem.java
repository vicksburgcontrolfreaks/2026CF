// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.TelemetryConstants;

public class CollectorSubsystem extends SubsystemBase {
  private final SparkFlex m_upperCollectorMotor;
  private final SparkFlex m_lowerCollectorMotor;
  private final SparkMax m_hopperMotor;
  private final DigitalInput m_limitSwitch;

  private double m_hopperTargetPosition = 0.0;
  private boolean m_lastSwitchState = false;
  private int m_telemetryCounter = 0;

  private final NetworkTable m_telemetryTable;
  private final DoublePublisher m_upperCollectorSpeedPub;
  private final DoublePublisher m_lowerCollectorSpeedPub;
  private final DoublePublisher m_hopperPositionPub;
  private final DoublePublisher m_hopperSpeedPub;
  private final DoublePublisher m_upperCollectorCurrentPub;
  private final DoublePublisher m_lowerCollectorCurrentPub;
  private final DoublePublisher m_hopperCurrentPub;
  private final BooleanPublisher m_limitSwitchPub;

  public CollectorSubsystem() {
    m_upperCollectorMotor = new SparkFlex(CollectorConstants.kUpperCollectorMotorId, MotorType.kBrushless);
    m_lowerCollectorMotor = new SparkFlex(CollectorConstants.kLowerCollectorMotorId, MotorType.kBrushless);
    m_hopperMotor = new SparkMax(CollectorConstants.kHopperMotorId, MotorType.kBrushless);
    m_limitSwitch = new DigitalInput(CollectorConstants.kLimitSwitchDIO);

    m_upperCollectorMotor.configure(CollectorConstants.collectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_lowerCollectorMotor.configure(CollectorConstants.collectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_hopperMotor.configure(CollectorConstants.hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_telemetryTable = NetworkTableInstance.getDefault().getTable("Collector");
    m_upperCollectorSpeedPub = m_telemetryTable.getDoubleTopic("Upper Collector Speed").publish();
    m_lowerCollectorSpeedPub = m_telemetryTable.getDoubleTopic("Lower Collector Speed").publish();
    m_hopperPositionPub = m_telemetryTable.getDoubleTopic("Hopper Position").publish();
    m_hopperSpeedPub = m_telemetryTable.getDoubleTopic("Hopper Speed").publish();
    m_upperCollectorCurrentPub = m_telemetryTable.getDoubleTopic("Upper Collector Current").publish();
    m_lowerCollectorCurrentPub = m_telemetryTable.getDoubleTopic("Lower Collector Current").publish();
    m_hopperCurrentPub = m_telemetryTable.getDoubleTopic("Hopper Current").publish();
    m_limitSwitchPub = m_telemetryTable.getBooleanTopic("Limit Switch").publish();
  }

  public void runCollector() {
    m_upperCollectorMotor.set(-CollectorConstants.kCollectorSpeed);
    m_lowerCollectorMotor.set(CollectorConstants.kCollectorSpeed);
  }

  public void stopCollector() {
    m_upperCollectorMotor.set(0);
    m_lowerCollectorMotor.set(0);
  }

  public void setHopperPosition(double position) {
    m_hopperTargetPosition = position;
  }

  public double getHopperPosition() {
    return m_hopperMotor.getEncoder().getPosition();
  }

  public boolean isHopperAtPosition(double targetPosition) {
    return Math.abs(getHopperPosition() - targetPosition) < CollectorConstants.kHopperPositionTolerance;
  }

  public boolean isLimitSwitchPressed() {
    return m_limitSwitch.get();
  }

  public void stopAll() {
    m_upperCollectorMotor.set(0);
    m_lowerCollectorMotor.set(0);
    m_hopperMotor.set(0);
  }

  private void updateHopperMotion() {
    m_hopperMotor.getClosedLoopController().setSetpoint(
      m_hopperTargetPosition,
      ControlType.kPosition
    );
  }

  @Override
  public void periodic() {
    boolean currentSwitchState = isLimitSwitchPressed();

    if (currentSwitchState && !m_lastSwitchState) {
      m_hopperMotor.getEncoder().setPosition(0.0);
      m_hopperTargetPosition = CollectorConstants.kHopperSwitchOffsetTicks;
    }

    m_lastSwitchState = currentSwitchState;

    updateHopperMotion();

    m_telemetryCounter++;
    if (m_telemetryCounter >= TelemetryConstants.kTelemetryUpdatePeriod) {
      m_telemetryCounter = 0;

      m_upperCollectorSpeedPub.set(m_upperCollectorMotor.get());
      m_lowerCollectorSpeedPub.set(m_lowerCollectorMotor.get());
      m_hopperPositionPub.set(getHopperPosition());
      m_hopperSpeedPub.set(m_hopperMotor.get());

      m_upperCollectorCurrentPub.set(m_upperCollectorMotor.getOutputCurrent());
      m_lowerCollectorCurrentPub.set(m_lowerCollectorMotor.getOutputCurrent());
      m_hopperCurrentPub.set(m_hopperMotor.getOutputCurrent());
      m_limitSwitchPub.set(currentSwitchState);
    }
  }
}

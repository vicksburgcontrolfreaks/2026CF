// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.TelemetryConstants;

public class CollectorSubsystem extends SubsystemBase {
  private final SparkFlex m_upperCollectorMotor;
  private final SparkFlex m_lowerCollectorMotor;
  private final SparkMax m_hopperMotor;

  private double m_hopperTargetPosition = 0.0;
  private int m_telemetryCounter = 0;

  private final NetworkTable m_telemetryTable;
  private final DoublePublisher m_upperCollectorSpeedPub;
  private final DoublePublisher m_lowerCollectorSpeedPub;
  private final DoublePublisher m_hopperPositionPub;
  private final DoublePublisher m_hopperSpeedPub;
  private final DoublePublisher m_upperCollectorCurrentPub;
  private final DoublePublisher m_lowerCollectorCurrentPub;
  private final DoublePublisher m_hopperCurrentPub;

  public CollectorSubsystem() {
    m_upperCollectorMotor = new SparkFlex(CollectorConstants.kUpperCollectorMotorId, MotorType.kBrushless);
    m_lowerCollectorMotor = new SparkFlex(CollectorConstants.kLowerCollectorMotorId, MotorType.kBrushless);
    m_hopperMotor = new SparkMax(CollectorConstants.kHopperMotorId, MotorType.kBrushless);

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
  }

  public void runCollector() {
    m_upperCollectorMotor.set(CollectorConstants.kCollectorSpeed);
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

  public void stopAll() {
    m_upperCollectorMotor.set(0);
    m_lowerCollectorMotor.set(0);
    m_hopperMotor.set(0);
  }

  private double calculateEasing(double progress) {
    progress = MathUtil.clamp(progress, 0.0, 1.0);

    if (progress < 0.5) {
      return 2 * progress * progress;
    } else {
      double p = progress - 1;
      return 1 - 2 * p * p;
    }
  }

  private void updateHopperMotion() {
    double currentPosition = getHopperPosition();
    double error = m_hopperTargetPosition - currentPosition;
    double distance = Math.abs(error);
    double maxDistance = Math.abs(CollectorConstants.kHopperDeployedPosition - CollectorConstants.kHopperRetractedPosition);

    if (distance < CollectorConstants.kHopperPositionTolerance) {
      m_hopperMotor.set(0);
      return;
    }

    double progress = 1.0 - (distance / maxDistance);
    double speedMultiplier = calculateEasing(progress);
    double speed = CollectorConstants.kHopperMaxSpeed * speedMultiplier;
    speed = Math.max(speed, 0.1);

    if (error < 0) {
      speed *= -1;
    }

    m_hopperMotor.set(speed);
  }

  @Override
  public void periodic() {
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
    }
  }
}

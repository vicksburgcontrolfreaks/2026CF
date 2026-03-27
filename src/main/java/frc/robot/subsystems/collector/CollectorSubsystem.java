// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.CollectorConfig;
import frc.robot.constants.CollectorConstants;
import frc.robot.constants.TelemetryConstants;

public class CollectorSubsystem extends SubsystemBase {
  private final SparkFlex m_upperCollectorMotor;
  private final SparkFlex m_lowerCollectorMotor;
  private final SparkMax m_hopperMotor;

  private int m_telemetryCounter = 0;

  private final NetworkTable m_telemetryTable;
  private final DoublePublisher m_upperCollectorSpeedPub;
  private final DoublePublisher m_lowerCollectorSpeedPub;
  private final DoublePublisher m_upperCollectorCurrentPub;
  private final DoublePublisher m_lowerCollectorCurrentPub;
  private final DoublePublisher m_hopperMotorPosiotionPub;


  public CollectorSubsystem() {
    m_upperCollectorMotor = new SparkFlex(CollectorConstants.kUpperCollectorMotorId, MotorType.kBrushless);
    m_lowerCollectorMotor = new SparkFlex(CollectorConstants.kLowerCollectorMotorId, MotorType.kBrushless);
    m_hopperMotor = new SparkMax(CollectorConstants.kHopperMotorId, MotorType.kBrushless);

    m_upperCollectorMotor.configure(CollectorConfig.collectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_lowerCollectorMotor.configure(CollectorConfig.collectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_hopperMotor.configure(CollectorConfig.hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set hopper to 0.07 position on enable (between up and halfway)
    m_hopperMotor.getClosedLoopController().setSetpoint(
      0.07,
      SparkMax.ControlType.kPosition
    );

    m_telemetryTable = NetworkTableInstance.getDefault().getTable("Collector");
    m_upperCollectorSpeedPub = m_telemetryTable.getDoubleTopic("Upper Collector Speed").publish();
    m_lowerCollectorSpeedPub = m_telemetryTable.getDoubleTopic("Lower Collector Speed").publish();
    m_upperCollectorCurrentPub = m_telemetryTable.getDoubleTopic("Upper Collector Current").publish();
    m_lowerCollectorCurrentPub = m_telemetryTable.getDoubleTopic("Lower Collector Current").publish();
    m_hopperMotorPosiotionPub = m_telemetryTable.getDoubleTopic("Hopper Position").publish();
  }

  public void runCollector(boolean reversed) {
    if (!reversed)
    {
      m_upperCollectorMotor.set(-CollectorConstants.kCollectorSpeed);
      // Lower collector disabled during collection to reduce battery usage
      // m_lowerCollectorMotor.set(-CollectorConstants.kCollectorSpeed);
    } else {
      m_upperCollectorMotor.set(CollectorConstants.kCollectorSpeed);
      // Lower collector disabled during collection to reduce battery usage
      // m_lowerCollectorMotor.set(CollectorConstants.kCollectorSpeed);
    }
  }

  /**
   * Run collector at a specific RPM using velocity control
   * @param rpm Target velocity in RPM (positive = forward, negative = reverse)
   */
  public void runCollectorRPM(double rpm) {
    m_upperCollectorMotor.getClosedLoopController().setSetpoint(
      -rpm,
      SparkFlex.ControlType.kVelocity
    );
    m_lowerCollectorMotor.getClosedLoopController().setSetpoint(
      -rpm,
      SparkFlex.ControlType.kVelocity
    );
  }

  /**
   * Run only lower collector at a specific RPM (for shooting sequence)
   * @param rpm Target velocity in RPM (positive = forward, negative = reverse)
   */
  public void runLowerCollectorRPM(double rpm) {
    m_lowerCollectorMotor.getClosedLoopController().setSetpoint(
      -rpm,
      SparkFlex.ControlType.kVelocity
    );
  }

  public void stopCollector() {
    m_upperCollectorMotor.set(0);
    m_lowerCollectorMotor.set(0);
  }

  public void retractHopper() {
    m_hopperMotor.getClosedLoopController().setSetpoint(
      CollectorConstants.kUpPosition,
      SparkMax.ControlType.kPosition
    );
  }

  /**
   * Retract hopper to halfway position slowly (for shooting sequence)
   * Uses position control with motion constraints configured in CollectorConfig
   */
  public void retractHopperHalfwaySlow() {
    // Move to halfway position using MAXMotion (motion profiling configured in CollectorConfig)
    m_hopperMotor.getClosedLoopController().setSetpoint(
      CollectorConstants.kHalfwayPosition,
      SparkMax.ControlType.kMAXMotionPositionControl
    );
  }

  public void extendHopper() {
    m_hopperMotor.getClosedLoopController().setSetpoint(
      CollectorConstants.kDownPosition,
      SparkMax.ControlType.kPosition
    );
  }

  public double getHopperPosition() {
    return m_hopperMotor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    m_telemetryCounter++;
    if (m_telemetryCounter >= TelemetryConstants.kTelemetryUpdatePeriod) {
      m_telemetryCounter = 0;

      m_upperCollectorSpeedPub.set(m_upperCollectorMotor.get());
      m_lowerCollectorSpeedPub.set(m_lowerCollectorMotor.get());
      m_upperCollectorCurrentPub.set(m_upperCollectorMotor.getOutputCurrent());
      m_lowerCollectorCurrentPub.set(m_lowerCollectorMotor.getOutputCurrent());
      m_hopperMotorPosiotionPub.set(m_hopperMotor.getAbsoluteEncoder().getPosition());
    }
    double CurrentHopperPose = m_hopperMotor.getAbsoluteEncoder().getPosition();
  }
}

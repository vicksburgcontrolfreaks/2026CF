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
import edu.wpi.first.networktables.DoubleSubscriber;
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

  // Configurable constants (via NetworkTables)
  private int m_motorCurrentLimit = CollectorConstants.kMotorCurrentLimit;
  private double m_collectorSpeed = CollectorConstants.kCollectorSpeed;
  private double m_upPosition = CollectorConstants.kUpPosition;
  private double m_downPosition = CollectorConstants.kDownPosition;
  private double m_hopperP = CollectorConstants.kHopperP;
  private double m_hopperI = CollectorConstants.kHopperI;
  private double m_hopperD = CollectorConstants.kHopperD;
  private int m_telemetryUpdatePeriod = TelemetryConstants.kTelemetryUpdatePeriod;

  // NetworkTables subscribers for configurable constants
  private final DoubleSubscriber m_motorCurrentLimitSub;
  private final DoubleSubscriber m_collectorSpeedSub;
  private final DoubleSubscriber m_upPositionSub;
  private final DoubleSubscriber m_downPositionSub;
  private final DoubleSubscriber m_hopperPSub;
  private final DoubleSubscriber m_hopperISub;
  private final DoubleSubscriber m_hopperDSub;
  private final DoubleSubscriber m_telemetryUpdatePeriodSub;


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

    // Initialize NetworkTables subscribers for configurable constants
    NetworkTable configTable = NetworkTableInstance.getDefault().getTable("Collector/Config");
    m_motorCurrentLimitSub = configTable.getDoubleTopic("Motor Current Limit").subscribe(m_motorCurrentLimit);
    m_collectorSpeedSub = configTable.getDoubleTopic("Collector Speed").subscribe(m_collectorSpeed);
    m_upPositionSub = configTable.getDoubleTopic("Up Position").subscribe(m_upPosition);
    m_downPositionSub = configTable.getDoubleTopic("Down Position").subscribe(m_downPosition);
    m_hopperPSub = configTable.getDoubleTopic("Hopper P").subscribe(m_hopperP);
    m_hopperISub = configTable.getDoubleTopic("Hopper I").subscribe(m_hopperI);
    m_hopperDSub = configTable.getDoubleTopic("Hopper D").subscribe(m_hopperD);
    m_telemetryUpdatePeriodSub = configTable.getDoubleTopic("Telemetry Update Period").subscribe(m_telemetryUpdatePeriod);
  }

  public void runCollector(boolean reversed) {
    if (!reversed)
    {
      m_upperCollectorMotor.set(-getCollectorSpeed());
      m_lowerCollectorMotor.set(-getCollectorSpeed());
    } else {
      m_upperCollectorMotor.set(getCollectorSpeed());
      m_lowerCollectorMotor.set(getCollectorSpeed());
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
      getUpPosition(),
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
      getDownPosition(),
      SparkMax.ControlType.kPosition
    );
  }

  public void setHopperPosition(double position) {
    m_hopperMotor.getClosedLoopController().setSetpoint(
      position,
      SparkMax.ControlType.kPosition
    );
  }

  public double getHopperPosition() {
    return m_hopperMotor.getAbsoluteEncoder().getPosition();
  }

  // Getters and setters for configurable constants
  public int getMotorCurrentLimit() {
    return m_motorCurrentLimit;
  }

  public void setMotorCurrentLimit(int limit) {
    m_motorCurrentLimit = limit;
  }

  public double getCollectorSpeed() {
    return m_collectorSpeed;
  }

  public void setCollectorSpeed(double speed) {
    m_collectorSpeed = speed;
  }

  public double getUpPosition() {
    return m_upPosition;
  }

  public void setUpPosition(double position) {
    m_upPosition = position;
  }

  public double getDownPosition() {
    return m_downPosition;
  }

  public void setDownPosition(double position) {
    m_downPosition = position;
  }

  public double getHopperP() {
    return m_hopperP;
  }

  public void setHopperP(double p) {
    m_hopperP = p;
    updateHopperPID();
  }

  public double getHopperI() {
    return m_hopperI;
  }

  public void setHopperI(double i) {
    m_hopperI = i;
    updateHopperPID();
  }

  public double getHopperD() {
    return m_hopperD;
  }

  public void setHopperD(double d) {
    m_hopperD = d;
    updateHopperPID();
  }

  public int getTelemetryUpdatePeriod() {
    return m_telemetryUpdatePeriod;
  }

  public void setTelemetryUpdatePeriod(int period) {
    m_telemetryUpdatePeriod = period;
  }

  /**
   * Update hopper motor PID values by reconfiguring motor
   */
  private void updateHopperPID() {
    com.revrobotics.spark.config.SparkMaxConfig config = new com.revrobotics.spark.config.SparkMaxConfig();
    config.closedLoop
      .pid(m_hopperP, m_hopperI, m_hopperD);

    m_hopperMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // Read configurable values from NetworkTables
    double newMotorCurrentLimit = m_motorCurrentLimitSub.get();
    if (newMotorCurrentLimit != m_motorCurrentLimit) {
      m_motorCurrentLimit = (int) newMotorCurrentLimit;
    }

    m_collectorSpeed = m_collectorSpeedSub.get();
    m_upPosition = m_upPositionSub.get();
    m_downPosition = m_downPositionSub.get();

    double newHopperP = m_hopperPSub.get();
    double newHopperI = m_hopperISub.get();
    double newHopperD = m_hopperDSub.get();
    if (newHopperP != m_hopperP || newHopperI != m_hopperI || newHopperD != m_hopperD) {
      m_hopperP = newHopperP;
      m_hopperI = newHopperI;
      m_hopperD = newHopperD;
      updateHopperPID();
    }

    m_telemetryUpdatePeriod = (int) m_telemetryUpdatePeriodSub.get();

    m_telemetryCounter++;
    if (m_telemetryCounter >= getTelemetryUpdatePeriod()) {
      m_telemetryCounter = 0;

      m_upperCollectorSpeedPub.set(m_upperCollectorMotor.get());
      m_lowerCollectorSpeedPub.set(m_lowerCollectorMotor.get());
      m_upperCollectorCurrentPub.set(m_upperCollectorMotor.getOutputCurrent());
      m_lowerCollectorCurrentPub.set(m_lowerCollectorMotor.getOutputCurrent());
      m_hopperMotorPosiotionPub.set(m_hopperMotor.getAbsoluteEncoder().getPosition());
    }
  }
}

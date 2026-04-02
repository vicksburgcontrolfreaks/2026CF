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
  // Lower collector motor removed - CAN ID 11 no longer in use
  private final SparkMax m_hopperMotor;

  private int m_telemetryCounter = 0;

  private final NetworkTable m_telemetryTable;
  private final DoublePublisher m_upperCollectorSpeedPub;
  private final DoublePublisher m_upperCollectorVelocityPub;
  private final DoublePublisher m_upperCollectorCurrentPub;
  private final DoublePublisher m_hopperMotorPosiotionPub;

  // Configurable constants (via NetworkTables)
  private int m_motorCurrentLimit = CollectorConstants.kMotorCurrentLimit;
  private double m_collectorSpeed = CollectorConstants.kCollectorSpeed;
  private double m_collectorTargetRPM = CollectorConstants.kCollectorTargetRPM;
  private double m_collectorP = CollectorConstants.kCollectorP;
  private double m_collectorI = CollectorConstants.kCollectorI;
  private double m_collectorD = CollectorConstants.kCollectorD;
  private double m_collectorFF = CollectorConstants.kCollectorFF;
  private double m_upPosition = CollectorConstants.kUpPosition;
  private double m_downPosition = CollectorConstants.kDownPosition;
  private double m_hopperP = CollectorConstants.kHopperP;
  private double m_hopperI = CollectorConstants.kHopperI;
  private double m_hopperD = CollectorConstants.kHopperD;
  private int m_telemetryUpdatePeriod = TelemetryConstants.kTelemetryUpdatePeriod;

  // NetworkTables subscribers for configurable constants
  private final DoubleSubscriber m_motorCurrentLimitSub;
  private final DoubleSubscriber m_collectorSpeedSub;
  private final DoubleSubscriber m_collectorTargetRPMSub;
  private final DoubleSubscriber m_collectorPSub;
  private final DoubleSubscriber m_collectorISub;
  private final DoubleSubscriber m_collectorDSub;
  private final DoubleSubscriber m_collectorFFSub;
  private final DoubleSubscriber m_upPositionSub;
  private final DoubleSubscriber m_downPositionSub;
  private final DoubleSubscriber m_hopperPSub;
  private final DoubleSubscriber m_hopperISub;
  private final DoubleSubscriber m_hopperDSub;
  private final DoubleSubscriber m_telemetryUpdatePeriodSub;

  // NetworkTables publishers for configurable constants (so values appear in dashboard)
  private final DoublePublisher m_motorCurrentLimitPub;
  private final DoublePublisher m_collectorSpeedPub;
  private final DoublePublisher m_collectorTargetRPMPub;
  private final DoublePublisher m_collectorPPub;
  private final DoublePublisher m_collectorIPub;
  private final DoublePublisher m_collectorDPub;
  private final DoublePublisher m_collectorFFPub;
  private final DoublePublisher m_upPositionPub;
  private final DoublePublisher m_downPositionPub;
  private final DoublePublisher m_hopperPPub;
  private final DoublePublisher m_hopperIPub;
  private final DoublePublisher m_hopperDPub;
  private final DoublePublisher m_telemetryUpdatePeriodPub;


  public CollectorSubsystem() {
    m_upperCollectorMotor = new SparkFlex(CollectorConstants.kUpperCollectorMotorId, MotorType.kBrushless);
    m_hopperMotor = new SparkMax(CollectorConstants.kHopperMotorId, MotorType.kBrushless);

    m_upperCollectorMotor.configure(CollectorConfig.collectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_hopperMotor.configure(CollectorConfig.hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set hopper to 0.07 position on enable (between up and halfway)
    m_hopperMotor.getClosedLoopController().setSetpoint(
      0.07,
      SparkMax.ControlType.kPosition
    );

    m_telemetryTable = NetworkTableInstance.getDefault().getTable("Collector");
    m_upperCollectorSpeedPub = m_telemetryTable.getDoubleTopic("Upper Collector Speed").publish();
    m_upperCollectorVelocityPub = m_telemetryTable.getDoubleTopic("Upper Collector Velocity (RPM)").publish();
    m_upperCollectorCurrentPub = m_telemetryTable.getDoubleTopic("Upper Collector Current").publish();
    m_hopperMotorPosiotionPub = m_telemetryTable.getDoubleTopic("Hopper Position").publish();

    // Initialize NetworkTables subscribers AND publishers for configurable constants
    NetworkTable configTable = NetworkTableInstance.getDefault().getTable("Collector/Config");

    // Create publishers and set initial values
    m_motorCurrentLimitPub = configTable.getDoubleTopic("Motor Current Limit").publish();
    m_collectorSpeedPub = configTable.getDoubleTopic("Collector Speed").publish();
    m_collectorTargetRPMPub = configTable.getDoubleTopic("Collector Target RPM").publish();
    m_collectorPPub = configTable.getDoubleTopic("Collector P").publish();
    m_collectorIPub = configTable.getDoubleTopic("Collector I").publish();
    m_collectorDPub = configTable.getDoubleTopic("Collector D").publish();
    m_collectorFFPub = configTable.getDoubleTopic("Collector FF").publish();
    m_upPositionPub = configTable.getDoubleTopic("Up Position").publish();
    m_downPositionPub = configTable.getDoubleTopic("Down Position").publish();
    m_hopperPPub = configTable.getDoubleTopic("Hopper P").publish();
    m_hopperIPub = configTable.getDoubleTopic("Hopper I").publish();
    m_hopperDPub = configTable.getDoubleTopic("Hopper D").publish();
    m_telemetryUpdatePeriodPub = configTable.getDoubleTopic("Telemetry Update Period").publish();

    m_motorCurrentLimitPub.set(m_motorCurrentLimit);
    m_collectorSpeedPub.set(m_collectorSpeed);
    m_collectorTargetRPMPub.set(m_collectorTargetRPM);
    m_collectorPPub.set(m_collectorP);
    m_collectorIPub.set(m_collectorI);
    m_collectorDPub.set(m_collectorD);
    m_collectorFFPub.set(m_collectorFF);
    m_upPositionPub.set(m_upPosition);
    m_downPositionPub.set(m_downPosition);
    m_hopperPPub.set(m_hopperP);
    m_hopperIPub.set(m_hopperI);
    m_hopperDPub.set(m_hopperD);
    m_telemetryUpdatePeriodPub.set(m_telemetryUpdatePeriod);

    // Subscribe to read updates from dashboard
    m_motorCurrentLimitSub = configTable.getDoubleTopic("Motor Current Limit").subscribe(m_motorCurrentLimit);
    m_collectorSpeedSub = configTable.getDoubleTopic("Collector Speed").subscribe(m_collectorSpeed);
    m_collectorTargetRPMSub = configTable.getDoubleTopic("Collector Target RPM").subscribe(m_collectorTargetRPM);
    m_collectorPSub = configTable.getDoubleTopic("Collector P").subscribe(m_collectorP);
    m_collectorISub = configTable.getDoubleTopic("Collector I").subscribe(m_collectorI);
    m_collectorDSub = configTable.getDoubleTopic("Collector D").subscribe(m_collectorD);
    m_collectorFFSub = configTable.getDoubleTopic("Collector FF").subscribe(m_collectorFF);
    m_upPositionSub = configTable.getDoubleTopic("Up Position").subscribe(m_upPosition);
    m_downPositionSub = configTable.getDoubleTopic("Down Position").subscribe(m_downPosition);
    m_hopperPSub = configTable.getDoubleTopic("Hopper P").subscribe(m_hopperP);
    m_hopperISub = configTable.getDoubleTopic("Hopper I").subscribe(m_hopperI);
    m_hopperDSub = configTable.getDoubleTopic("Hopper D").subscribe(m_hopperD);
    m_telemetryUpdatePeriodSub = configTable.getDoubleTopic("Telemetry Update Period").subscribe(m_telemetryUpdatePeriod);
  }

  public void runCollector(boolean reversed) {
    // Use velocity control with target RPM
    double targetRPM = reversed ? -getCollectorTargetRPM() : getCollectorTargetRPM();
    m_upperCollectorMotor.getClosedLoopController().setSetpoint(
      -targetRPM,  // Negative because motor is mounted reversed
      SparkFlex.ControlType.kVelocity
    );
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
  }

  public void stopCollector() {
    m_upperCollectorMotor.set(0);
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

  public double getCollectorTargetRPM() {
    return m_collectorTargetRPM;
  }

  public void setCollectorTargetRPM(double rpm) {
    m_collectorTargetRPM = rpm;
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

  /**
   * Update collector motor PID + FF values by reconfiguring motor
   */
  private void updateCollectorPID() {
    com.revrobotics.spark.config.SparkFlexConfig config = new com.revrobotics.spark.config.SparkFlexConfig();
    config.closedLoop
      .pid(m_collectorP, m_collectorI, m_collectorD)
      .velocityFF(m_collectorFF);

    m_upperCollectorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // Read configurable values from NetworkTables and update if changed
    double newMotorCurrentLimit = m_motorCurrentLimitSub.get();
    if (newMotorCurrentLimit != m_motorCurrentLimit) {
      m_motorCurrentLimit = (int) newMotorCurrentLimit;
      m_motorCurrentLimitPub.set(m_motorCurrentLimit);
    }

    double newCollectorSpeed = m_collectorSpeedSub.get();
    if (newCollectorSpeed != m_collectorSpeed) {
      m_collectorSpeed = newCollectorSpeed;
      m_collectorSpeedPub.set(m_collectorSpeed);
    }

    double newCollectorTargetRPM = m_collectorTargetRPMSub.get();
    if (newCollectorTargetRPM != m_collectorTargetRPM) {
      m_collectorTargetRPM = newCollectorTargetRPM;
      m_collectorTargetRPMPub.set(m_collectorTargetRPM);
    }

    // Read collector PID values and update motor if changed
    double newCollectorP = m_collectorPSub.get();
    double newCollectorI = m_collectorISub.get();
    double newCollectorD = m_collectorDSub.get();
    double newCollectorFF = m_collectorFFSub.get();
    if (newCollectorP != m_collectorP || newCollectorI != m_collectorI ||
        newCollectorD != m_collectorD || newCollectorFF != m_collectorFF) {
      m_collectorP = newCollectorP;
      m_collectorI = newCollectorI;
      m_collectorD = newCollectorD;
      m_collectorFF = newCollectorFF;
      m_collectorPPub.set(m_collectorP);
      m_collectorIPub.set(m_collectorI);
      m_collectorDPub.set(m_collectorD);
      m_collectorFFPub.set(m_collectorFF);
      updateCollectorPID();
    }

    double newUpPosition = m_upPositionSub.get();
    if (newUpPosition != m_upPosition) {
      m_upPosition = newUpPosition;
      m_upPositionPub.set(m_upPosition);
    }

    double newDownPosition = m_downPositionSub.get();
    if (newDownPosition != m_downPosition) {
      m_downPosition = newDownPosition;
      m_downPositionPub.set(m_downPosition);
    }

    double newHopperP = m_hopperPSub.get();
    double newHopperI = m_hopperISub.get();
    double newHopperD = m_hopperDSub.get();
    if (newHopperP != m_hopperP || newHopperI != m_hopperI || newHopperD != m_hopperD) {
      m_hopperP = newHopperP;
      m_hopperI = newHopperI;
      m_hopperD = newHopperD;
      m_hopperPPub.set(m_hopperP);
      m_hopperIPub.set(m_hopperI);
      m_hopperDPub.set(m_hopperD);
      updateHopperPID();
    }

    double newTelemetryUpdatePeriod = m_telemetryUpdatePeriodSub.get();
    if (newTelemetryUpdatePeriod != m_telemetryUpdatePeriod) {
      m_telemetryUpdatePeriod = (int) newTelemetryUpdatePeriod;
      m_telemetryUpdatePeriodPub.set(m_telemetryUpdatePeriod);
    }

    m_telemetryCounter++;
    if (m_telemetryCounter >= getTelemetryUpdatePeriod()) {
      m_telemetryCounter = 0;

      m_upperCollectorSpeedPub.set(m_upperCollectorMotor.get());
      m_upperCollectorVelocityPub.set(m_upperCollectorMotor.getEncoder().getVelocity());
      m_upperCollectorCurrentPub.set(m_upperCollectorMotor.getOutputCurrent());
      m_hopperMotorPosiotionPub.set(m_hopperMotor.getAbsoluteEncoder().getPosition());
    }
  }
}

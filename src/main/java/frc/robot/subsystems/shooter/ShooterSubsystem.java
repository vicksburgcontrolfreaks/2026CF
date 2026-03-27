// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ShooterConfig;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TelemetryConstants;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex m_floorMotor;
  private final SparkFlex m_leftIndexerMotor;
  private final SparkFlex m_rightIndexerMotor;
  private final SparkFlex m_middleIndexerMotor;
  private final SparkFlex m_rightShooterMotor;
  private final SparkFlex m_leftShooterMotor;
  private final SparkFlex m_middleShooterMotor;

  private int m_telemetryCounter = 0;

  private final NetworkTable m_telemetryTable;
  private final DoublePublisher m_rightShooterSpeedPub;
  private final DoublePublisher m_floorMotorSpeedPub;
  private final DoublePublisher m_leftIndexerSpeedPub;
  private final DoublePublisher m_rightIndexerSpeedPub;
  private final DoublePublisher m_middleIndexerSpeedPub;
  private final DoublePublisher m_leftShooterSpeedPub;
  private final DoublePublisher m_middleShooterSpeedPub;
  private final DoublePublisher m_rightShooterCurrentPub;
  private final DoublePublisher m_floorMotorCurrentPub;
  private final DoublePublisher m_leftIndexerCurrentPub;
  private final DoublePublisher m_rightIndexerCurrentPub;
  private final DoublePublisher m_middleIndexerCurrentPub;
  private final DoublePublisher m_leftShooterCurrentPub;
  private final DoublePublisher m_middleShooterCurrentPub;
  private final DoublePublisher m_rightShooterVelocityPub;
  private final DoublePublisher m_floorMotorVelocityPub;
  private final DoublePublisher m_leftIndexerVelocityPub;
  private final DoublePublisher m_rightIndexerVelocityPub;
  private final DoublePublisher m_middleIndexerVelocityPub;
  private final DoublePublisher m_leftShooterVelocityPub;
  private final DoublePublisher m_middleShooterVelocityPub;
  private final DoublePublisher m_targetRPMPub;
  private final DoublePublisher m_distanceToTargetPub;
  private final StringPublisher m_debugMessagePub;

  private double m_currentTargetRPM = ShooterConstants.kShooterTargetRPM;
  private double m_calculatedTargetRPM = ShooterConstants.kShooterTargetRPM; // Always reflects linear regression calculation
  private double m_lastDistanceToTarget = 0.0;
  private boolean m_isShooterActive = false; // Track if shooter is actively running
  private final PhotonVisionSubsystem m_visionSubsystem;

  // Configurable constants (via NetworkTables)
  private int m_motorCurrentLimit = ShooterConstants.kMotorCurrentLimit;
  private double m_shooterP = ShooterConstants.kShooterP;
  private double m_shooterI = ShooterConstants.kShooterI;
  private double m_shooterD = ShooterConstants.kShooterD;
  private double m_shooterFF = ShooterConstants.kShooterFF;
  private double m_indexerP = ShooterConstants.kIndexerP;
  private double m_indexerI = ShooterConstants.kIndexerI;
  private double m_indexerD = ShooterConstants.kIndexerD;
  private double m_indexerFF = ShooterConstants.kIndexerFF;
  private double m_shooterTargetRPM = ShooterConstants.kShooterTargetRPM;
  private double m_floorMotorTargetRPM = ShooterConstants.kFloorMotorTargetRPM;
  private double m_indexerMotorTargetRPM = ShooterConstants.kIndexerMotorTargetRPM;
  private double[][] m_shooterVelocityTable = ShooterConstants.kShooterVelocityTable;
  private int m_telemetryUpdatePeriod = TelemetryConstants.kTelemetryUpdatePeriod;

  // NetworkTables subscribers for configurable constants
  private final DoubleSubscriber m_motorCurrentLimitSub;
  private final DoubleSubscriber m_shooterPSub;
  private final DoubleSubscriber m_shooterISub;
  private final DoubleSubscriber m_shooterDSub;
  private final DoubleSubscriber m_shooterFFSub;
  private final DoubleSubscriber m_indexerPSub;
  private final DoubleSubscriber m_indexerISub;
  private final DoubleSubscriber m_indexerDSub;
  private final DoubleSubscriber m_indexerFFSub;
  private final DoubleSubscriber m_shooterTargetRPMSub;
  private final DoubleSubscriber m_floorMotorTargetRPMSub;
  private final DoubleSubscriber m_indexerMotorTargetRPMSub;
  private final DoubleSubscriber m_telemetryUpdatePeriodSub;

  //private static double kTargetRPM = 3000; // 40% of max velocity
  // max rpm 6784 

  public ShooterSubsystem(PhotonVisionSubsystem visionSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_floorMotor = new SparkFlex(ShooterConstants.kFloorMotorId, MotorType.kBrushless);
    m_leftShooterMotor = new SparkFlex(ShooterConstants.kLeftShooterId, MotorType.kBrushless);
    m_rightShooterMotor = new SparkFlex(ShooterConstants.kRightShooterId, MotorType.kBrushless);
    m_middleShooterMotor = new SparkFlex(ShooterConstants.kMiddleShooterId, MotorType.kBrushless);
    m_leftIndexerMotor = new SparkFlex(ShooterConstants.kLIndexerMotorId, MotorType.kBrushless);
    m_rightIndexerMotor = new SparkFlex(ShooterConstants.kRIndexerMotorId, MotorType.kBrushless);
    m_middleIndexerMotor = new SparkFlex(ShooterConstants.kMIndexerMotorId, MotorType.kBrushless);


    m_leftShooterMotor.configure(ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightShooterMotor.configure(ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_middleShooterMotor.configure(ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_floorMotor.configure(ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftIndexerMotor.configure(ShooterConfig.indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightIndexerMotor.configure(ShooterConfig.indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_middleIndexerMotor.configure(ShooterConfig.indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    m_leftIndexerMotor.set(0);
    m_rightIndexerMotor.set(0);
    m_middleIndexerMotor.set(0);
    m_floorMotor.set(0);

    m_telemetryTable = NetworkTableInstance.getDefault().getTable("Shooter");
    m_rightShooterSpeedPub   = m_telemetryTable.getDoubleTopic("Right Shooter Speed").publish();
    m_floorMotorSpeedPub     = m_telemetryTable.getDoubleTopic("Floor Motor Speed").publish();
    m_leftIndexerSpeedPub    = m_telemetryTable.getDoubleTopic("Left Indexer Speed").publish();
    m_rightIndexerSpeedPub   = m_telemetryTable.getDoubleTopic("Right Indexer Speed").publish();
    m_middleIndexerSpeedPub  = m_telemetryTable.getDoubleTopic("Middle Indexer Speed").publish();
    m_leftShooterSpeedPub    = m_telemetryTable.getDoubleTopic("Left Shooter Speed").publish();
    m_middleShooterSpeedPub  = m_telemetryTable.getDoubleTopic("Middle Shooter Speed").publish();
    m_rightShooterCurrentPub = m_telemetryTable.getDoubleTopic("Right Shooter Current").publish();
    m_floorMotorCurrentPub   = m_telemetryTable.getDoubleTopic("Floor Motor Current").publish();
    m_leftIndexerCurrentPub  = m_telemetryTable.getDoubleTopic("Left Indexer Current").publish();
    m_rightIndexerCurrentPub = m_telemetryTable.getDoubleTopic("Right Indexer Current").publish();
    m_middleIndexerCurrentPub = m_telemetryTable.getDoubleTopic("Middle Indexer Current").publish();
    m_leftShooterCurrentPub  = m_telemetryTable.getDoubleTopic("Left Shooter Current").publish();
    m_middleShooterCurrentPub = m_telemetryTable.getDoubleTopic("Middle Shooter Current").publish();
    m_rightShooterVelocityPub  = m_telemetryTable.getDoubleTopic("Right Shooter Velocity").publish();
    m_floorMotorVelocityPub    = m_telemetryTable.getDoubleTopic("Floor Motor Velocity").publish();
    m_leftIndexerVelocityPub   = m_telemetryTable.getDoubleTopic("Left Indexer Velocity").publish();
    m_rightIndexerVelocityPub  = m_telemetryTable.getDoubleTopic("Right Indexer Velocity").publish();
    m_middleIndexerVelocityPub = m_telemetryTable.getDoubleTopic("Middle Indexer Velocity").publish();
    m_leftShooterVelocityPub   = m_telemetryTable.getDoubleTopic("Left Shooter Velocity").publish();
    m_middleShooterVelocityPub = m_telemetryTable.getDoubleTopic("Middle Shooter Velocity").publish();
    m_targetRPMPub             = m_telemetryTable.getDoubleTopic("Target RPM").publish();
    m_distanceToTargetPub      = m_telemetryTable.getDoubleTopic("Distance to Target").publish();
    m_debugMessagePub          = m_telemetryTable.getStringTopic("Debug Message").publish();

    // Initialize NetworkTables subscribers for configurable constants
    NetworkTable configTable = NetworkTableInstance.getDefault().getTable("Shooter/Config");
    m_motorCurrentLimitSub = configTable.getDoubleTopic("Motor Current Limit").subscribe(m_motorCurrentLimit);
    m_shooterPSub = configTable.getDoubleTopic("Shooter P").subscribe(m_shooterP);
    m_shooterISub = configTable.getDoubleTopic("Shooter I").subscribe(m_shooterI);
    m_shooterDSub = configTable.getDoubleTopic("Shooter D").subscribe(m_shooterD);
    m_shooterFFSub = configTable.getDoubleTopic("Shooter FF").subscribe(m_shooterFF);
    m_indexerPSub = configTable.getDoubleTopic("Indexer P").subscribe(m_indexerP);
    m_indexerISub = configTable.getDoubleTopic("Indexer I").subscribe(m_indexerI);
    m_indexerDSub = configTable.getDoubleTopic("Indexer D").subscribe(m_indexerD);
    m_indexerFFSub = configTable.getDoubleTopic("Indexer FF").subscribe(m_indexerFF);
    m_shooterTargetRPMSub = configTable.getDoubleTopic("Shooter Target RPM").subscribe(m_shooterTargetRPM);
    m_floorMotorTargetRPMSub = configTable.getDoubleTopic("Floor Motor Target RPM").subscribe(m_floorMotorTargetRPM);
    m_indexerMotorTargetRPMSub = configTable.getDoubleTopic("Indexer Motor Target RPM").subscribe(m_indexerMotorTargetRPM);
    m_telemetryUpdatePeriodSub = configTable.getDoubleTopic("Telemetry Update Period").subscribe(m_telemetryUpdatePeriod);
  }

  public void runFloor(boolean reversed) {
    double targetRPM = getFloorMotorTargetRPM();
    if (reversed) {
      targetRPM = -targetRPM;
    }

    m_floorMotor.getClosedLoopController().setSetpoint(
      targetRPM,
      ControlType.kVelocity
    );
  }

  public void StopFloor() {
    m_floorMotor.set(0);
  }

  public void StopIndexer() {
    m_leftIndexerMotor.set(0);
    m_rightIndexerMotor.set(0);
    m_middleIndexerMotor.set(0);
  }

  /**
   * Calculate target RPM based on distance to speaker using linear interpolation
   * @param distance Distance to speaker in meters
   * @return Target RPM for the shooter
   */
  private double getRPMForDistance(double distance) {
    double[][] table = getShooterVelocityTable();
    if (table.length == 0) {
      return getShooterTargetRPM();
    }

    if (distance <= table[0][0]) {
      return table[0][1];
    }

    int lastIndex = table.length - 1;
    if (distance >= table[lastIndex][0]) {
      return table[lastIndex][1];
    }

    for (int i = 0; i < table.length - 1; i++) {
      double d1 = table[i][0];
      double rpm1 = table[i][1];
      double d2 = table[i + 1][0];
      double rpm2 = table[i + 1][1];

      if (distance >= d1 && distance <= d2) {
        return rpm1 + (rpm2 - rpm1) * (distance - d1) / (d2 - d1);
      }
    }

    return getShooterTargetRPM();
  }

  /**
   * Activate the shooter motors. The RPM will be continuously updated by periodic()
   * based on the robot's distance to the speaker (when dynamic RPM is enabled).
   * This allows shooting while moving - the RPM adjusts in real-time.
   */
  public void activateShooter() {
    m_isShooterActive = true;
    m_currentTargetRPM = m_calculatedTargetRPM;

    // Immediately command motors to start spinning up
    m_leftShooterMotor.getClosedLoopController().setSetpoint(
      m_currentTargetRPM,
      ControlType.kVelocity
    );

    m_rightShooterMotor.getClosedLoopController().setSetpoint(
      m_currentTargetRPM,
      ControlType.kVelocity
    );

    m_middleShooterMotor.getClosedLoopController().setSetpoint(
      m_currentTargetRPM,
      ControlType.kVelocity
    );
  }

  /**
   * Stop the shooter motors (set to 0).
   */
  public void stopShooter() {
    m_isShooterActive = false;

    m_leftShooterMotor.set(0);
    m_rightShooterMotor.set(0);
    m_middleShooterMotor.set(0);
  }

  /**
   * Get the calculated target RPM (from linear regression)
   * This is the RPM calculated by periodic() based on vision distance,
   * NOT necessarily the RPM currently being commanded to the motors.
   * @return The calculated target RPM from linear regression
   */
  public double getCurrentTargetRPM() {
    return m_calculatedTargetRPM;
  }

  /**
   * Get the RPM currently being commanded to the motors
   * @return The motor command RPM
   */
  public double getMotorTargetRPM() {
    return m_currentTargetRPM;
  }

  /**
   * Check if the shooter is currently active
   * @return True if shooter is active and running
   */
  public boolean isShooterActive() {
    return m_isShooterActive;
  }

  /**
   * Check if the shooter is at the target velocity
   * @param tolerance Acceptable RPM tolerance (e.g., 100 RPM)
   * @return True if both shooter motors are within tolerance of target
   */
  public boolean isAtTargetVelocity(double tolerance) {
    double leftVelocity = Math.abs(m_leftShooterMotor.getEncoder().getVelocity());
    double rightVelocity = Math.abs(m_rightShooterMotor.getEncoder().getVelocity());
    double middleVelocity = Math.abs(m_middleShooterMotor.getEncoder().getVelocity());

    return Math.abs(leftVelocity - m_currentTargetRPM) <= tolerance &&
           Math.abs(rightVelocity - m_currentTargetRPM) <= tolerance && 
           Math.abs(middleVelocity - m_currentTargetRPM) <= tolerance;
  }

  public void runIndexer(boolean reversed) {
    double rpm = getIndexerMotorTargetRPM();
    if (reversed) {
      rpm = -rpm;
    }

    m_leftIndexerMotor.getClosedLoopController().setSetpoint(
      rpm,
      ControlType.kVelocity
    );

    m_rightIndexerMotor.getClosedLoopController().setSetpoint(
      rpm,
      ControlType.kVelocity
    );

    m_middleIndexerMotor.getClosedLoopController().setSetpoint(
      -rpm,
      ControlType.kVelocity
    );
  }

  public boolean isAnyMotorOverCurrent(double threshold) {
    return m_floorMotor.getOutputCurrent() > threshold ||

           m_leftIndexerMotor.getOutputCurrent() > threshold ||
           m_rightIndexerMotor.getOutputCurrent() > threshold ||
           m_middleIndexerMotor.getOutputCurrent() > threshold ||

           m_leftShooterMotor.getOutputCurrent() > threshold ||
           m_rightShooterMotor.getOutputCurrent() > threshold ||
           m_middleShooterMotor.getOutputCurrent() > threshold;
  }

  // Getters and setters for configurable constants
  public int getMotorCurrentLimit() {
    return m_motorCurrentLimit;
  }

  public void setMotorCurrentLimit(int limit) {
    m_motorCurrentLimit = limit;
  }

  public double getShooterP() {
    return m_shooterP;
  }

  public void setShooterP(double p) {
    m_shooterP = p;
    updateShooterPID();
  }

  public double getShooterI() {
    return m_shooterI;
  }

  public void setShooterI(double i) {
    m_shooterI = i;
    updateShooterPID();
  }

  public double getShooterD() {
    return m_shooterD;
  }

  public void setShooterD(double d) {
    m_shooterD = d;
    updateShooterPID();
  }

  public double getShooterFF() {
    return m_shooterFF;
  }

  public void setShooterFF(double ff) {
    m_shooterFF = ff;
    updateShooterPID();
  }

  public double getIndexerP() {
    return m_indexerP;
  }

  public void setIndexerP(double p) {
    m_indexerP = p;
    updateIndexerPID();
  }

  public double getIndexerI() {
    return m_indexerI;
  }

  public void setIndexerI(double i) {
    m_indexerI = i;
    updateIndexerPID();
  }

  public double getIndexerD() {
    return m_indexerD;
  }

  public void setIndexerD(double d) {
    m_indexerD = d;
    updateIndexerPID();
  }

  public double getIndexerFF() {
    return m_indexerFF;
  }

  public void setIndexerFF(double ff) {
    m_indexerFF = ff;
    updateIndexerPID();
  }

  public double getShooterTargetRPM() {
    return m_shooterTargetRPM;
  }

  public void setShooterTargetRPM(double rpm) {
    m_shooterTargetRPM = rpm;
  }

  public double getFloorMotorTargetRPM() {
    return m_floorMotorTargetRPM;
  }

  public void setFloorMotorTargetRPM(double rpm) {
    m_floorMotorTargetRPM = rpm;
  }

  public double getIndexerMotorTargetRPM() {
    return m_indexerMotorTargetRPM;
  }

  public void setIndexerMotorTargetRPM(double rpm) {
    m_indexerMotorTargetRPM = rpm;
  }

  public double[][] getShooterVelocityTable() {
    return m_shooterVelocityTable;
  }

  public void setShooterVelocityTable(double[][] table) {
    m_shooterVelocityTable = table;
  }

  public int getTelemetryUpdatePeriod() {
    return m_telemetryUpdatePeriod;
  }

  public void setTelemetryUpdatePeriod(int period) {
    m_telemetryUpdatePeriod = period;
  }

  /**
   * Update shooter motor PID values by reconfiguring motors
   */
  private void updateShooterPID() {
    com.revrobotics.spark.config.SparkFlexConfig config = new com.revrobotics.spark.config.SparkFlexConfig();
    config.closedLoop
      .pid(m_shooterP, m_shooterI, m_shooterD)
      .velocityFF(m_shooterFF)
      .outputRange(-1, 1);

    m_leftShooterMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightShooterMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_middleShooterMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Update indexer motor PID values by reconfiguring motors
   */
  private void updateIndexerPID() {
    com.revrobotics.spark.config.SparkFlexConfig config = new com.revrobotics.spark.config.SparkFlexConfig();
    config.closedLoop
      .pid(m_indexerP, m_indexerI, m_indexerD)
      .velocityFF(m_indexerFF)
      .outputRange(-1, 1);

    m_leftIndexerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightIndexerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_middleIndexerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // Read configurable values from NetworkTables
    double newMotorCurrentLimit = m_motorCurrentLimitSub.get();
    if (newMotorCurrentLimit != m_motorCurrentLimit) {
      m_motorCurrentLimit = (int) newMotorCurrentLimit;
    }

    double newShooterP = m_shooterPSub.get();
    double newShooterI = m_shooterISub.get();
    double newShooterD = m_shooterDSub.get();
    double newShooterFF = m_shooterFFSub.get();
    if (newShooterP != m_shooterP || newShooterI != m_shooterI ||
        newShooterD != m_shooterD || newShooterFF != m_shooterFF) {
      m_shooterP = newShooterP;
      m_shooterI = newShooterI;
      m_shooterD = newShooterD;
      m_shooterFF = newShooterFF;
      updateShooterPID();
    }

    double newIndexerP = m_indexerPSub.get();
    double newIndexerI = m_indexerISub.get();
    double newIndexerD = m_indexerDSub.get();
    double newIndexerFF = m_indexerFFSub.get();
    if (newIndexerP != m_indexerP || newIndexerI != m_indexerI ||
        newIndexerD != m_indexerD || newIndexerFF != m_indexerFF) {
      m_indexerP = newIndexerP;
      m_indexerI = newIndexerI;
      m_indexerD = newIndexerD;
      m_indexerFF = newIndexerFF;
      updateIndexerPID();
    }

    m_shooterTargetRPM = m_shooterTargetRPMSub.get();
    m_floorMotorTargetRPM = m_floorMotorTargetRPMSub.get();
    m_indexerMotorTargetRPM = m_indexerMotorTargetRPMSub.get();
    m_telemetryUpdatePeriod = (int) m_telemetryUpdatePeriodSub.get();

    // DYNAMIC RPM ENABLED - Calculate RPM based on vision distance
    // ALWAYS calculate target RPM based on vision distance (continuously runs linear regression)
    // This updates m_calculatedTargetRPM which is published to Elastic dashboard
    if (m_visionSubsystem != null) {
      double distance = m_visionSubsystem.getDistanceToSpeaker();
      m_lastDistanceToTarget = distance;

      if (distance > 0) {
        m_calculatedTargetRPM = getRPMForDistance(distance);
      } else {
        // Fall back to default RPM if distance calculation fails
        m_calculatedTargetRPM = getShooterTargetRPM();
      }
    } else {
      // No vision subsystem, use default
      m_calculatedTargetRPM = getShooterTargetRPM();
    }

    // ALWAYS publish Target RPM and Distance (no throttling) so Elastic updates in real-time
    m_targetRPMPub.set(m_calculatedTargetRPM);
    m_distanceToTargetPub.set(m_lastDistanceToTarget);

    // If shooter is active, continuously update motor commands with calculated RPM
    if (m_isShooterActive) {
      m_currentTargetRPM = m_calculatedTargetRPM;

      m_leftShooterMotor.getClosedLoopController().setSetpoint(
        m_currentTargetRPM,
        ControlType.kVelocity
      );

      m_rightShooterMotor.getClosedLoopController().setSetpoint(
        m_currentTargetRPM,
        ControlType.kVelocity
      );

      m_middleShooterMotor.getClosedLoopController().setSetpoint(
        -m_currentTargetRPM,
        ControlType.kVelocity
      );
    }

    m_telemetryCounter++;
    if (m_telemetryCounter >= getTelemetryUpdatePeriod()) {
      m_telemetryCounter = 0;

      m_floorMotorSpeedPub.set(m_floorMotor.get());
      m_leftIndexerSpeedPub.set(m_leftIndexerMotor.get());
      m_rightIndexerSpeedPub.set(m_rightIndexerMotor.get());
      m_middleIndexerSpeedPub.set(m_middleIndexerMotor.get());
      m_leftShooterSpeedPub.set(m_leftShooterMotor.get());
      m_rightShooterSpeedPub.set(m_rightShooterMotor.get());
      m_middleShooterSpeedPub.set(m_middleShooterMotor.get());

      m_rightShooterCurrentPub.set(m_rightShooterMotor.getOutputCurrent());
      m_floorMotorCurrentPub.set(m_floorMotor.getOutputCurrent());
      m_leftIndexerCurrentPub.set(m_leftIndexerMotor.getOutputCurrent());
      m_rightIndexerCurrentPub.set(m_rightIndexerMotor.getOutputCurrent());
      m_middleIndexerCurrentPub.set(m_middleIndexerMotor.getOutputCurrent());
      m_leftShooterCurrentPub.set(m_leftShooterMotor.getOutputCurrent());
      m_middleShooterCurrentPub.set(m_middleShooterMotor.getOutputCurrent());

      m_rightShooterVelocityPub.set(m_rightShooterMotor.getEncoder().getVelocity());
      m_floorMotorVelocityPub.set(m_floorMotor.getEncoder().getVelocity());
      m_leftIndexerVelocityPub.set(m_leftIndexerMotor.getEncoder().getVelocity());
      m_rightIndexerVelocityPub.set(m_rightIndexerMotor.getEncoder().getVelocity());
      m_middleIndexerVelocityPub.set(m_middleIndexerMotor.getEncoder().getVelocity());
      m_leftShooterVelocityPub.set(m_leftShooterMotor.getEncoder().getVelocity());
      m_middleShooterVelocityPub.set(m_middleShooterMotor.getEncoder().getVelocity());
    }
  }

  public static double getKShooterTargetRPM() {
    return ShooterConstants.kShooterTargetRPM;
  }
}

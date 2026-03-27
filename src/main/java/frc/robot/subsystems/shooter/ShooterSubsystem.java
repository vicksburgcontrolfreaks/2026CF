// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.networktables.DoublePublisher;
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
  private boolean m_useRPMCap = true; // Cap RPM at kPreSpinRPMCap until trigger pulled
  private final PhotonVisionSubsystem m_visionSubsystem;
  private frc.robot.RobotContainer m_container;  // For accessing PID tuning entries

  // Cache last applied PID values to avoid reconfiguring every cycle
  private double m_lastShooterP = ShooterConstants.kShooterP;
  private double m_lastShooterI = ShooterConstants.kShooterI;
  private double m_lastShooterD = ShooterConstants.kShooterD;
  private double m_lastShooterFF = ShooterConstants.kShooterFF;
  private double m_lastIndexerP = ShooterConstants.kIndexerP;
  private double m_lastIndexerI = ShooterConstants.kIndexerI;
  private double m_lastIndexerD = ShooterConstants.kIndexerD;
  private double m_lastIndexerFF = ShooterConstants.kIndexerFF;

  // Velocity capture system for graphing during shooting
  private static final int CAPTURE_BUFFER_SIZE = 15; // 15 samples at 20ms = 300ms
  private static final double VELOCITY_DIP_THRESHOLD = 200.0; // RPM drop to trigger capture
  private final double[] m_leftVelocityBuffer = new double[CAPTURE_BUFFER_SIZE];
  private final double[] m_rightVelocityBuffer = new double[CAPTURE_BUFFER_SIZE];
  private final double[] m_middleVelocityBuffer = new double[CAPTURE_BUFFER_SIZE];
  private final double[] m_timestampBuffer = new double[CAPTURE_BUFFER_SIZE];
  private int m_bufferIndex = 0;
  private boolean m_isCapturing = false;
  private int m_captureRemaining = 0;
  private double m_lastVelocity = 0.0;
  private double m_captureStartTime = 0.0;
  private int m_captureCount = 0;
  private final DoublePublisher[] m_capturedLeftVelocityPubs = new DoublePublisher[CAPTURE_BUFFER_SIZE];
  private final DoublePublisher[] m_capturedRightVelocityPubs = new DoublePublisher[CAPTURE_BUFFER_SIZE];
  private final DoublePublisher[] m_capturedMiddleVelocityPubs = new DoublePublisher[CAPTURE_BUFFER_SIZE];
  private final DoublePublisher[] m_capturedTimestampPubs = new DoublePublisher[CAPTURE_BUFFER_SIZE];
  private final DoublePublisher m_captureCountPub;

  //private static double kTargetRPM = 3000; // 40% of max velocity
  // max rpm 6784

  public ShooterSubsystem(PhotonVisionSubsystem visionSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_container = null;  // Will be set later via setContainer()
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

    // Initialize velocity capture publishers
    NetworkTable captureTable = NetworkTableInstance.getDefault().getTable("ShooterVelocityCapture");
    for (int i = 0; i < CAPTURE_BUFFER_SIZE; i++) {
      m_capturedLeftVelocityPubs[i] = captureTable.getDoubleTopic("LeftVelocity" + i).publish();
      m_capturedRightVelocityPubs[i] = captureTable.getDoubleTopic("RightVelocity" + i).publish();
      m_capturedMiddleVelocityPubs[i] = captureTable.getDoubleTopic("MiddleVelocity" + i).publish();
      m_capturedTimestampPubs[i] = captureTable.getDoubleTopic("Timestamp" + i).publish();
    }
    m_captureCountPub = captureTable.getDoubleTopic("CaptureCount").publish();
  }

  public void runFloor(boolean reversed) {
    double targetRPM = ShooterConstants.kFloorMotorTargetRPM;
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
    if (distance <= ShooterConstants.kShooterVelocityTable[0][0]) {
      return ShooterConstants.kShooterVelocityTable[0][1];
    }

    int lastIndex = ShooterConstants.kShooterVelocityTable.length - 1;
    if (distance >= ShooterConstants.kShooterVelocityTable[lastIndex][0]) {
      return ShooterConstants.kShooterVelocityTable[lastIndex][1];
    }

    for (int i = 0; i < ShooterConstants.kShooterVelocityTable.length - 1; i++) {
      double d1 = ShooterConstants.kShooterVelocityTable[i][0];
      double rpm1 = ShooterConstants.kShooterVelocityTable[i][1];
      double d2 = ShooterConstants.kShooterVelocityTable[i + 1][0];
      double rpm2 = ShooterConstants.kShooterVelocityTable[i + 1][1];

      if (distance >= d1 && distance <= d2) {
        return rpm1 + (rpm2 - rpm1) * (distance - d1) / (d2 - d1);
      }
    }

    return ShooterConstants.kShooterTargetRPM;
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
   * Activate the shooter motors with a specific RPM (for testing)
   * @param rpm Target RPM for all shooter motors
   */
  public void activateShooterWithRPM(double rpm) {
    m_isShooterActive = true;
    m_currentTargetRPM = rpm;

    m_leftShooterMotor.getClosedLoopController().setSetpoint(
      rpm,
      ControlType.kVelocity
    );

    m_rightShooterMotor.getClosedLoopController().setSetpoint(
      rpm,
      ControlType.kVelocity
    );

    m_middleShooterMotor.getClosedLoopController().setSetpoint(
      rpm,
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
    double rpm = ShooterConstants.kIndexerMotorTargetRPM;
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

  /**
   * Run indexer with specific RPM (for testing)
   * @param rpm Target RPM for indexer motors
   * @param reversed True to reverse direction
   */
  public void runIndexerWithRPM(double rpm, boolean reversed) {
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

  @Override
  public void periodic() {
    // Update motor PID from NetworkTables (allows real-time tuning)
    updateMotorPIDFromNetworkTables();

    // DYNAMIC RPM ENABLED - Calculate target RPM based on vision distance
    // ALWAYS calculate target RPM based on vision distance (continuously runs linear regression)
    // This updates m_calculatedTargetRPM which is published to Elastic dashboard
    if (m_visionSubsystem != null) {
      double distance = m_visionSubsystem.getDistanceToSpeaker();
      m_lastDistanceToTarget = distance;

      if (distance > 0) {
        m_calculatedTargetRPM = getRPMForDistance(distance);
      } else {
        // Fall back to default RPM if distance calculation fails
        m_calculatedTargetRPM = ShooterConstants.kShooterTargetRPM;
      }
    } else {
      // No vision subsystem, use default
      m_calculatedTargetRPM = ShooterConstants.kShooterTargetRPM;
    }

    // ALWAYS publish Target RPM and Distance (no throttling) so Elastic updates in real-time
    m_targetRPMPub.set(m_calculatedTargetRPM);
    m_distanceToTargetPub.set(m_lastDistanceToTarget);

    // If shooter is active, continuously update motor commands with calculated RPM
    if (m_isShooterActive) {
      // Apply pre-spin RPM cap if enabled (until trigger is pulled)
      if (m_useRPMCap) {
        m_currentTargetRPM = Math.min(m_calculatedTargetRPM, ShooterConstants.kPreSpinRPMCap);
      } else {
        m_currentTargetRPM = m_calculatedTargetRPM;
      }

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

    // Velocity capture system - capture data when velocity dips (ball contact)
    captureVelocityData();

    m_telemetryCounter++;
    if (m_telemetryCounter >= TelemetryConstants.kTelemetryUpdatePeriod) {
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

  // ========== Test Telemetry Methods ==========

  /**
   * Get average shooter RPM across all three shooter motors
   * @return Average actual RPM
   */
  public double getAverageShooterRPM() {
    double leftRPM = Math.abs(m_leftShooterMotor.getEncoder().getVelocity());
    double rightRPM = Math.abs(m_rightShooterMotor.getEncoder().getVelocity());
    double middleRPM = Math.abs(m_middleShooterMotor.getEncoder().getVelocity());
    return (leftRPM + rightRPM + middleRPM) / 3.0;
  }

  /**
   * Get average indexer RPM across all three indexer motors
   * @return Average actual RPM
   */
  public double getAverageIndexerRPM() {
    double leftRPM = Math.abs(m_leftIndexerMotor.getEncoder().getVelocity());
    double rightRPM = Math.abs(m_rightIndexerMotor.getEncoder().getVelocity());
    double middleRPM = Math.abs(m_middleIndexerMotor.getEncoder().getVelocity());
    return (leftRPM + rightRPM + middleRPM) / 3.0;
  }

  /**
   * Get average shooter current across all three shooter motors
   * @return Average current in amps
   */
  public double getAverageShooterCurrent() {
    double leftCurrent = m_leftShooterMotor.getOutputCurrent();
    double rightCurrent = m_rightShooterMotor.getOutputCurrent();
    double middleCurrent = m_middleShooterMotor.getOutputCurrent();
    return (leftCurrent + rightCurrent + middleCurrent) / 3.0;
  }

  /**
   * Get average indexer current across all three indexer motors
   * @return Average current in amps
   */
  public double getAverageIndexerCurrent() {
    double leftCurrent = m_leftIndexerMotor.getOutputCurrent();
    double rightCurrent = m_rightIndexerMotor.getOutputCurrent();
    double middleCurrent = m_middleIndexerMotor.getOutputCurrent();
    return (leftCurrent + rightCurrent + middleCurrent) / 3.0;
  }

  /**
   * Get distance to speaker from vision subsystem
   * @return Distance in meters, or -1 if unavailable
   */
  public double getDistanceToSpeaker() {
    if (m_visionSubsystem != null) {
      return m_visionSubsystem.getDistanceToSpeaker();
    }
    return -1.0;
  }

  /**
   * Check if shooter is ready to feed balls (RPM at target)
   * Note: Commands should also check robot alignment before feeding
   * @return True if shooter velocity is within tolerance
   */
  public boolean isReadyToFeed() {
    return isShooterActive() && isAtTargetVelocity(100.0);
  }

  /**
   * Set the RobotContainer reference for accessing PID tuning entries
   * @param container RobotContainer instance
   */
  public void setContainer(frc.robot.RobotContainer container) {
    m_container = container;
  }

  /**
   * Enable full RPM (remove pre-spin cap).
   * Call this when trigger is pulled to allow distance-based RPM without limit.
   */
  public void enableFullRPM() {
    m_useRPMCap = false;
  }

  /**
   * Enable pre-spin RPM cap (limit to kPreSpinRPMCap).
   * Call this when trigger is released to conserve energy during pre-spin.
   */
  public void enableRPMCap() {
    m_useRPMCap = true;
  }

  /**
   * Capture velocity data for graphing. Triggers on velocity dip (ball contact).
   * Captures 300ms of data (15 samples at 20ms intervals) for all three shooter motors.
   */
  private void captureVelocityData() {
    // Get current velocities for all three motors
    double leftVelocity = m_leftShooterMotor.getEncoder().getVelocity();
    double rightVelocity = m_rightShooterMotor.getEncoder().getVelocity();
    double middleVelocity = Math.abs(m_middleShooterMotor.getEncoder().getVelocity()); // Abs because it spins opposite
    double averageVelocity = (leftVelocity + rightVelocity) / 2.0;
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    // Check if we should trigger a new capture
    if (!m_isCapturing && m_isShooterActive) {
      // Detect velocity dip (ball contact) using average velocity
      double velocityDrop = m_lastVelocity - averageVelocity;

      if (velocityDrop > VELOCITY_DIP_THRESHOLD && m_lastVelocity > m_currentTargetRPM * 0.8) {
        // Trigger new capture
        m_isCapturing = true;
        m_captureRemaining = CAPTURE_BUFFER_SIZE;
        m_bufferIndex = 0;
        m_captureStartTime = currentTime;
        m_captureCount++;
        System.out.println("Velocity capture triggered! Drop: " + velocityDrop + " RPM. Capture #" + m_captureCount);
      }
    }

    // If capturing, store data for all three motors
    if (m_isCapturing && m_captureRemaining > 0) {
      m_leftVelocityBuffer[m_bufferIndex] = leftVelocity;
      m_rightVelocityBuffer[m_bufferIndex] = rightVelocity;
      m_middleVelocityBuffer[m_bufferIndex] = middleVelocity;
      m_timestampBuffer[m_bufferIndex] = (currentTime - m_captureStartTime) * 1000.0; // Convert to ms
      m_bufferIndex++;
      m_captureRemaining--;

      // If capture complete, publish all data
      if (m_captureRemaining == 0) {
        publishCapturedData();
        m_isCapturing = false;
        System.out.println("Velocity capture complete. Published " + CAPTURE_BUFFER_SIZE + " samples for 3 motors.");
      }
    }

    m_lastVelocity = averageVelocity;
  }

  /**
   * Publish captured velocity data to NetworkTables for graphing in ShuffleBoard
   * Publishes data for all three shooter motors separately
   */
  private void publishCapturedData() {
    for (int i = 0; i < CAPTURE_BUFFER_SIZE; i++) {
      m_capturedLeftVelocityPubs[i].set(m_leftVelocityBuffer[i]);
      m_capturedRightVelocityPubs[i].set(m_rightVelocityBuffer[i]);
      m_capturedMiddleVelocityPubs[i].set(m_middleVelocityBuffer[i]);
      m_capturedTimestampPubs[i].set(m_timestampBuffer[i]);
    }
    m_captureCountPub.set(m_captureCount);
  }

  /**
   * Update motor PID gains from NetworkTables (for real-time tuning)
   * Only reconfigures motors when values actually change to avoid disrupting control loop
   */
  private void updateMotorPIDFromNetworkTables() {
    if (m_container == null) {
      return;  // Container not set yet, use default values
    }

    // Read shooter motor PID values
    double shooterP = m_container.m_shooterPEntry.get();
    double shooterI = m_container.m_shooterIEntry.get();
    double shooterD = m_container.m_shooterDEntry.get();
    double shooterFF = m_container.m_shooterFFEntry.get();

    // Read indexer motor PID values
    double indexerP = m_container.m_indexerPEntry.get();
    double indexerI = m_container.m_indexerIEntry.get();
    double indexerD = m_container.m_indexerDEntry.get();
    double indexerFF = m_container.m_indexerFFEntry.get();

    // Only update shooter motors if values changed
    if (shooterP != m_lastShooterP || shooterI != m_lastShooterI ||
        shooterD != m_lastShooterD || shooterFF != m_lastShooterFF) {

      SparkFlexConfig shooterConfig = new SparkFlexConfig();
      shooterConfig.closedLoop
        .pid(shooterP, shooterI, shooterD)
        .velocityFF(shooterFF);

      m_leftShooterMotor.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_rightShooterMotor.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_middleShooterMotor.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      // Update cached values
      m_lastShooterP = shooterP;
      m_lastShooterI = shooterI;
      m_lastShooterD = shooterD;
      m_lastShooterFF = shooterFF;

      System.out.println("Updated shooter PID: P=" + shooterP + " I=" + shooterI + " D=" + shooterD + " FF=" + shooterFF);
    }

    // Only update indexer motors if values changed
    if (indexerP != m_lastIndexerP || indexerI != m_lastIndexerI ||
        indexerD != m_lastIndexerD || indexerFF != m_lastIndexerFF) {

      SparkFlexConfig indexerConfig = new SparkFlexConfig();
      indexerConfig.closedLoop
        .pid(indexerP, indexerI, indexerD)
        .velocityFF(indexerFF);

      m_leftIndexerMotor.configure(indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_rightIndexerMotor.configure(indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_middleIndexerMotor.configure(indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      // Update cached values
      m_lastIndexerP = indexerP;
      m_lastIndexerI = indexerI;
      m_lastIndexerD = indexerD;
      m_lastIndexerFF = indexerFF;

      System.out.println("Updated indexer PID: P=" + indexerP + " I=" + indexerI + " D=" + indexerD + " FF=" + indexerFF);
    }
  }
}

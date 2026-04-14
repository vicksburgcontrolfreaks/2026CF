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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ShooterConfig;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TelemetryConstants;
import frc.robot.constants.TrajectoryTestConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
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

  private double m_currentTargetRPM = ShooterConstants.kShooterTargetRPM;
  private double m_calculatedTargetRPM = ShooterConstants.kShooterTargetRPM; // Always reflects linear regression calculation
  private double m_lastDistanceToTarget = 0.0;
  private boolean m_isShooterActive = false; // Track if shooter is actively running
  private boolean m_useRPMCap = true; // Cap RPM at kPreSpinRPMCap until trigger pulled
  private final PhotonVisionSubsystem m_visionSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final frc.robot.dashboard.ShooterDashboard m_dashboard;

  // Cache last applied PID values to avoid reconfiguring every cycle
  private double m_lastShooterP = ShooterConstants.kShooterP;
  private double m_lastShooterI = ShooterConstants.kShooterI;
  private double m_lastShooterD = ShooterConstants.kShooterD;
  private double m_lastShooterFF = ShooterConstants.kShooterFF;
  private double m_lastIndexerP = ShooterConstants.kIndexerP;
  private double m_lastIndexerI = ShooterConstants.kIndexerI;
  private double m_lastIndexerD = ShooterConstants.kIndexerD;
  private double m_lastIndexerFF = ShooterConstants.kIndexerFF;
  private double m_lastFloorMotorP = ShooterConstants.kFloorMotorP;
  private double m_lastFloorMotorI = ShooterConstants.kFloorMotorI;
  private double m_lastFloorMotorD = ShooterConstants.kFloorMotorD;
  private double m_lastFloorMotorFF = ShooterConstants.kFloorMotorFF;
  private int m_lastFloorMotorCurrentLimit = ShooterConstants.kFloorMotorCurrentLimit;

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

  // Live-tunable shooter velocity table (updated from dashboard)
  private double[][] m_shooterVelocityTable = ShooterConstants.kShooterVelocityTable;

  // Velocity compensation parameters
  private boolean m_velocityCompensationEnabled = ShooterConstants.kVelocityCompensationEnabled;
  private double m_angleCompensationFactor = ShooterConstants.kAngleCompensationFactor;
  private double m_averageShotVelocity = ShooterConstants.kAverageShotVelocity;

  // Trajectory test parameters
  private double m_trajectoryAngle = TrajectoryTestConstants.kBaselineTrajectoryAngle;
  private double m_testRPM = TrajectoryTestConstants.kTestRPMDefault;
  private boolean m_testModeEnabled = false;

  // Live-tunable pre-spin RPM cap (updated from dashboard)
  private double m_preSpinRPMCap = ShooterConstants.kPreSpinRPMCap;

  // RPM rate limiting to prevent regenerative braking spikes
  private double m_lastCommandedRPM = 0.0;

  // Floor motor jam detection and auto-recovery state machine
  private enum FloorMotorState { NORMAL, JAM_DETECTED, REVERSING, RECOVERY }
  private FloorMotorState m_floorMotorState = FloorMotorState.NORMAL;
  private double m_jamReverseStartTime = 0.0;
  private double m_savedFloorMotorRPM = 0.0;
  private boolean m_floorMotorWasRunning = false;
  private boolean m_floorMotorCommandedToRun = false;  // Track if we commanded motor to run
  private double m_lastJamRecoveryTime = 0.0;  // Timestamp of last recovery to prevent rapid re-triggering
  private static final double JAM_RECOVERY_COOLDOWN = 2.0;  // Wait 2 seconds after recovery before detecting again

  //private static double kTargetRPM = 3000; // 40% of max velocity
  // max rpm 6784

  public ShooterSubsystem(PhotonVisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_driveSubsystem = driveSubsystem;

    // Get dashboard instance from DashboardManager
    m_dashboard = frc.robot.dashboard.DashboardManager.getInstance().getShooterDashboard();
    m_floorMotor = new SparkFlex(ShooterConstants.kFloorMotorId, MotorType.kBrushless);
    m_leftShooterMotor = new SparkFlex(ShooterConstants.kLeftShooterId, MotorType.kBrushless);
    m_rightShooterMotor = new SparkFlex(ShooterConstants.kRightShooterId, MotorType.kBrushless);
    m_middleShooterMotor = new SparkFlex(ShooterConstants.kMiddleShooterId, MotorType.kBrushless);
    m_leftIndexerMotor = new SparkFlex(ShooterConstants.kLIndexerMotorId, MotorType.kBrushless);
    m_rightIndexerMotor = new SparkFlex(ShooterConstants.kRIndexerMotorId, MotorType.kBrushless);
    m_middleIndexerMotor = new SparkFlex(ShooterConstants.kMIndexerMotorId, MotorType.kBrushless);


    // Configure floor motor for velocity control with PID
    // Uses higher current limit (80A) than shooter wheels to handle mechanical load
    SparkFlexConfig floorMotorConfig = new SparkFlexConfig();
    floorMotorConfig
        .inverted(false)
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kFloorMotorCurrentLimit)
        .closedLoop
          .pid(ShooterConstants.kFloorMotorP, ShooterConstants.kFloorMotorI, ShooterConstants.kFloorMotorD)
          .velocityFF(ShooterConstants.kFloorMotorFF);
    m_floorMotor.configure(floorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure left shooter with inverted direction
    SparkFlexConfig leftShooterConfig = new SparkFlexConfig();
    leftShooterConfig
        .inverted(true)
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kMotorCurrentLimit)
        .closedLoop
          .pid(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD)
          .velocityFF(ShooterConstants.kShooterFF);
    m_leftShooterMotor.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure right shooter with inverted direction
    SparkFlexConfig rightShooterConfig = new SparkFlexConfig();
    rightShooterConfig
        .inverted(true)
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kMotorCurrentLimit)
        .closedLoop
          .pid(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD)
          .velocityFF(ShooterConstants.kShooterFF);
    m_rightShooterMotor.configure(rightShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure middle shooter with inverted direction
    SparkFlexConfig middleShooterConfig = new SparkFlexConfig();
    middleShooterConfig
        .inverted(true)
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kMotorCurrentLimit)
        .closedLoop
          .pid(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD)
          .velocityFF(ShooterConstants.kShooterFF);
    m_middleShooterMotor.configure(middleShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure left indexer (not inverted)
    m_leftIndexerMotor.configure(ShooterConfig.indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure right indexer with inverted direction
    SparkFlexConfig rightIndexerConfig = new SparkFlexConfig();
    rightIndexerConfig
        .inverted(true)
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kMotorCurrentLimit)
        .closedLoop
          .pid(ShooterConstants.kIndexerP, ShooterConstants.kIndexerI, ShooterConstants.kIndexerD)
          .velocityFF(ShooterConstants.kIndexerFF);
    m_rightIndexerMotor.configure(rightIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure middle indexer with inverted direction
    SparkFlexConfig middleIndexerConfig = new SparkFlexConfig();
    middleIndexerConfig
        .inverted(true)
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kMotorCurrentLimit)
        .closedLoop
          .pid(ShooterConstants.kIndexerP, ShooterConstants.kIndexerI, ShooterConstants.kIndexerD)
          .velocityFF(ShooterConstants.kIndexerFF);
    m_middleIndexerMotor.configure(middleIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_leftIndexerMotor.set(0);
    m_rightIndexerMotor.set(0);
    m_middleIndexerMotor.set(0);
    m_floorMotor.set(0);

    // Initialize velocity capture publishers for debugging shooting performance
    NetworkTable captureTable = NetworkTableInstance.getDefault().getTable("Shooter/Capture");
    for (int i = 0; i < CAPTURE_BUFFER_SIZE; i++) {
      m_capturedLeftVelocityPubs[i] = captureTable.getDoubleTopic("Left Velocity/" + i).publish();
      m_capturedRightVelocityPubs[i] = captureTable.getDoubleTopic("Right Velocity/" + i).publish();
      m_capturedMiddleVelocityPubs[i] = captureTable.getDoubleTopic("Middle Velocity/" + i).publish();
      m_capturedTimestampPubs[i] = captureTable.getDoubleTopic("Timestamp/" + i).publish();
    }
    m_captureCountPub = captureTable.getDoubleTopic("Count").publish();
  }

  public void runFloor(boolean reversed) {
    // Run floor motor with velocity control at target RPM
    double rpm = ShooterConstants.kFloorMotorTargetRPM;
    if (reversed) {
      rpm = -rpm;
    }
    m_floorMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    m_floorMotorCommandedToRun = true;  // Track that we commanded the motor to run
  }

  public void runFloorSlow(boolean reversed) {
    // Run floor motor at reduced velocity for slow feeding during collection
    double rpm = ShooterConstants.kFloorMotorTargetRPM * 0.3; // 30% of target RPM
    if (!reversed) {  // Note: reversed logic - slow mode runs opposite direction
      rpm = -rpm;
    }
    m_floorMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    m_floorMotorCommandedToRun = true;  // Track that we commanded the motor to run
  }

  /**
   * Run floor motor at a specific RPM using velocity control
   * @param rpm Target velocity in RPM (positive = forward, negative = reverse)
   */
  public void runFloorRPM(double rpm) {
    m_floorMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    m_floorMotorCommandedToRun = true;  // Track that we commanded the motor to run
  }

  public void StopFloor() {
    m_floorMotor.set(0);
    m_floorMotorCommandedToRun = false;  // Track that we stopped the motor
  }

  public void StopIndexer() {
    m_leftIndexerMotor.set(0);
    m_rightIndexerMotor.set(0);
    m_middleIndexerMotor.set(0);
  }

  /**
   * Handle floor motor jam detection and automatic recovery.
   *
   * State machine:
   * - NORMAL: Monitor for jam conditions (low velocity + high current)
   * - JAM_DETECTED: Save current state and transition to reversing
   * - REVERSING: Run motor in reverse for kFloorJamReverseTime seconds
   * - RECOVERY: Return to previous state (running or stopped)
   *
   * This prevents game pieces from getting stuck by automatically clearing jams.
   */
  private void handleFloorMotorJamDetection() {
    // Get current floor motor velocity for jam detection
    double floorVelocity = Math.abs(m_floorMotor.getEncoder().getVelocity());

    switch (m_floorMotorState) {
      case NORMAL:
        // Check for jam: low velocity indicates motor is stalled
        // Use our command tracking instead of SparkFlex setpoint (which may be cleared by controller)
        // Apply cooldown period after recovery to prevent rapid re-triggering
        double timeSinceLastRecovery = Timer.getFPGATimestamp() - m_lastJamRecoveryTime;
        boolean cooldownExpired = timeSinceLastRecovery > JAM_RECOVERY_COOLDOWN;

        // Only enable jam detection when running at high speed (shooting, not collection)
        // Collection runs at 1500 RPM, shooting at 2500 RPM - only detect jams during shooting
        double currentSetpoint = m_floorMotor.getClosedLoopController().getSetpoint();
        boolean runningAtShootingSpeed = Math.abs(currentSetpoint) > 2000; // High speed (shooting mode)

        if (m_floorMotorCommandedToRun &&
            runningAtShootingSpeed &&
            floorVelocity < ShooterConstants.kFloorJamVelocityThreshold &&
            cooldownExpired) {

          // Jam detected! Save current state
          m_savedFloorMotorRPM = currentSetpoint;
          m_floorMotorWasRunning = true;
          m_floorMotorState = FloorMotorState.JAM_DETECTED;

          System.out.println("FLOOR JAM DETECTED! Velocity: " + floorVelocity + " RPM (target: " + ShooterConstants.kFloorMotorTargetRPM + " RPM, current setpoint: " + currentSetpoint + " RPM)");
        }
        break;

      case JAM_DETECTED:
        // Transition immediately to reversing
        // Use duty cycle control (percent output) instead of velocity control
        // This applies full reverse power even when jammed, unlike velocity control which backs off
        m_jamReverseStartTime = Timer.getFPGATimestamp();
        m_floorMotor.set(-0.5);  // 50% reverse power (negative = reverse direction)
        m_floorMotorState = FloorMotorState.REVERSING;
        System.out.println("Starting jam reversal at 50% reverse power");
        break;

      case REVERSING:
        // Check if reverse time has elapsed
        double reverseElapsed = Timer.getFPGATimestamp() - m_jamReverseStartTime;
        if (reverseElapsed >= ShooterConstants.kFloorJamReverseTime) {
          m_floorMotorState = FloorMotorState.RECOVERY;
          System.out.println("Jam reversal complete, returning to previous state");
        }
        break;

      case RECOVERY:
        // Return to previous state
        if (m_floorMotorWasRunning) {
          // Restore previous setpoint
          m_floorMotor.getClosedLoopController().setSetpoint(
            m_savedFloorMotorRPM,
            ControlType.kVelocity
          );
          System.out.println("Restored floor motor to " + m_savedFloorMotorRPM + " RPM");
        } else {
          // Motor was stopped
          StopFloor();
          System.out.println("Stopped floor motor (was not running before jam)");
        }

        // Record recovery time and reset to normal state
        m_lastJamRecoveryTime = Timer.getFPGATimestamp();
        m_floorMotorState = FloorMotorState.NORMAL;
        System.out.println("Jam recovery complete. Cooldown active for " + JAM_RECOVERY_COOLDOWN + " seconds.");
        break;
    }
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
   * Activate only left and right shooters (disable middle shooter for hardware issue workaround)
   */
  public void activateLeftRightShootersOnly() {
    m_isShooterActive = true;
    m_currentTargetRPM = m_calculatedTargetRPM;

    // Run left and right shooters
    m_leftShooterMotor.getClosedLoopController().setSetpoint(
      m_currentTargetRPM,
      ControlType.kVelocity
    );

    m_rightShooterMotor.getClosedLoopController().setSetpoint(
      m_currentTargetRPM,
      ControlType.kVelocity
    );

    // Disable middle shooter
    m_middleShooterMotor.set(0);
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

    // Left indexer runs in one direction
    m_leftIndexerMotor.getClosedLoopController().setSetpoint(
      rpm,
      ControlType.kVelocity
    );

    // Right and middle indexers run in opposite direction (reversed)
    m_rightIndexerMotor.getClosedLoopController().setSetpoint(
      -rpm,
      ControlType.kVelocity
    );

    m_middleIndexerMotor.getClosedLoopController().setSetpoint(
      -rpm,
      ControlType.kVelocity
    );
  }

  /**
   * Run left/right indexers forward, middle indexer in reverse (hardware issue workaround)
   * Used during shooting sequence to clear middle path
   */
  public void runIndexerMiddleReversed() {
    double rpm = ShooterConstants.kIndexerMotorTargetRPM;

    // Run left indexer forward
    m_leftIndexerMotor.getClosedLoopController().setSetpoint(
      rpm,
      ControlType.kVelocity
    );

    // Run right indexer in reverse (opposite of left)
    m_rightIndexerMotor.getClosedLoopController().setSetpoint(
      -rpm,
      ControlType.kVelocity
    );

    // Run middle indexer in extra reverse (same direction as right)
    m_middleIndexerMotor.getClosedLoopController().setSetpoint(
      rpm,
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

    // Left indexer runs in one direction
    m_leftIndexerMotor.getClosedLoopController().setSetpoint(
      rpm,
      ControlType.kVelocity
    );

    // Right and middle indexers run in opposite direction (reversed)
    m_rightIndexerMotor.getClosedLoopController().setSetpoint(
      -rpm,
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

  // Getter for velocity table (used by getRPMForDistance)
  public double[][] getShooterVelocityTable() {
    return m_shooterVelocityTable;
  }

  // Getter for target RPM (used as fallback when distance unavailable)
  public double getShooterTargetRPM() {
    return ShooterConstants.kShooterTargetRPM;
  }

  @Override
  public void periodic() {
    // BEST PRACTICE: Read all configuration from dashboard in one place
    // This is cleaner than scattered NetworkTables.get() calls throughout the method
    var config = m_dashboard.readConfiguration();

    // Update PID values if changed (handled by updateMotorPIDFromDashboard)
    updateMotorPIDFromDashboard();

    // Update velocity table if changed
    if (!java.util.Arrays.deepEquals(config.velocityTable, m_shooterVelocityTable)) {
      m_shooterVelocityTable = config.velocityTable;
    }

    // Update other configuration values
    m_preSpinRPMCap = config.preSpinRPMCap;
    m_velocityCompensationEnabled = config.velocityCompensationEnabled;
    m_angleCompensationFactor = config.angleCompensationFactor;
    m_averageShotVelocity = config.averageShotVelocity;
    m_testModeEnabled = config.testModeEnabled;
    m_testRPM = config.testRPM;
    m_trajectoryAngle = config.trajectoryAngle;

    // DYNAMIC RPM ENABLED - Calculate RPM based on vision distance
    // ALWAYS calculate target RPM based on vision distance (continuously runs linear regression)
    // This updates m_calculatedTargetRPM which is published to Elastic dashboard
    if (m_visionSubsystem != null) {
      double distance = m_visionSubsystem.getDistanceToSpeaker();
      m_lastDistanceToTarget = distance;

      if (distance > 0) {
        m_calculatedTargetRPM = getVelocityCompensatedRPM(distance);
      } else {
        // Fall back to default RPM if distance calculation fails
        m_calculatedTargetRPM = getShooterTargetRPM();
      }
    } else {
      // No vision subsystem, use default
      m_calculatedTargetRPM = getShooterTargetRPM();
    }

    // If shooter is active, continuously update motor commands with calculated RPM
    if (m_isShooterActive) {
      // Determine target RPM: test mode overrides distance-based calculation
      if (m_testModeEnabled) {
        m_currentTargetRPM = m_testRPM;
        System.out.println("TEST MODE: Using test RPM = " + m_testRPM);
      } else {
        // Apply pre-spin RPM cap if enabled (until trigger is pulled)
        if (m_useRPMCap) {
          m_currentTargetRPM = Math.min(m_calculatedTargetRPM, m_preSpinRPMCap);
        } else {
          m_currentTargetRPM = m_calculatedTargetRPM;
        }
      }

      // Apply RPM rate limiting to prevent regenerative braking current spikes
      // Limits how quickly RPM can change per cycle (prevents 4300->2000 in 0.4s = 149A spike)
      double rpmDelta = m_currentTargetRPM - m_lastCommandedRPM;
      if (Math.abs(rpmDelta) > ShooterConstants.kMaxRPMChangePerCycle) {
        // Limit the change rate to prevent dangerous current spikes
        m_currentTargetRPM = m_lastCommandedRPM + Math.signum(rpmDelta) * ShooterConstants.kMaxRPMChangePerCycle;
      }
      m_lastCommandedRPM = m_currentTargetRPM;

      // Note: Target RPM and distance telemetry now published by dashboard

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

    // Floor motor jam detection and auto-recovery
    handleFloorMotorJamDetection();

    // Velocity capture system - capture data when velocity dips (ball contact)
    captureVelocityData();

    // BEST PRACTICE: Throttle telemetry updates to avoid excessive NetworkTables traffic
    m_telemetryCounter++;
    if (m_telemetryCounter >= TelemetryConstants.kTelemetryUpdatePeriod) {
      m_telemetryCounter = 0;

      // Update dashboard telemetry (all motor velocities, currents, and status)
      m_dashboard.updateTelemetry(this);
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

  // ========== Individual Motor Telemetry Getters (for Dashboard) ==========

  /**
   * Get left shooter motor velocity
   * @return Velocity in RPM
   */
  public double getLeftShooterVelocity() {
    return m_leftShooterMotor.getEncoder().getVelocity();
  }

  /**
   * Get right shooter motor velocity
   * @return Velocity in RPM
   */
  public double getRightShooterVelocity() {
    return m_rightShooterMotor.getEncoder().getVelocity();
  }

  /**
   * Get middle shooter motor velocity
   * @return Velocity in RPM
   */
  public double getMiddleShooterVelocity() {
    return m_middleShooterMotor.getEncoder().getVelocity();
  }

  /**
   * Get left shooter motor current
   * @return Current in amps
   */
  public double getLeftShooterCurrent() {
    return m_leftShooterMotor.getOutputCurrent();
  }

  /**
   * Get right shooter motor current
   * @return Current in amps
   */
  public double getRightShooterCurrent() {
    return m_rightShooterMotor.getOutputCurrent();
  }

  /**
   * Get middle shooter motor current
   * @return Current in amps
   */
  public double getMiddleShooterCurrent() {
    return m_middleShooterMotor.getOutputCurrent();
  }

  /**
   * Get floor motor velocity
   * @return Velocity in RPM
   */
  public double getFloorMotorVelocity() {
    return m_floorMotor.getEncoder().getVelocity();
  }

  /**
   * Get floor motor current
   * @return Current in amps
   */
  public double getFloorMotorCurrent() {
    return m_floorMotor.getOutputCurrent();
  }

  /**
   * Get floor motor velocity setpoint
   * @return Setpoint in RPM
   */
  public double getFloorMotorSetpoint() {
    return m_floorMotor.getClosedLoopController().getSetpoint();
  }

  /**
   * Get left indexer motor velocity
   * @return Velocity in RPM
   */
  public double getLeftIndexerVelocity() {
    return m_leftIndexerMotor.getEncoder().getVelocity();
  }

  /**
   * Get right indexer motor velocity
   * @return Velocity in RPM
   */
  public double getRightIndexerVelocity() {
    return m_rightIndexerMotor.getEncoder().getVelocity();
  }

  /**
   * Get middle indexer motor velocity
   * @return Velocity in RPM
   */
  public double getMiddleIndexerVelocity() {
    return m_middleIndexerMotor.getEncoder().getVelocity();
  }

  /**
   * Get left indexer motor current
   * @return Current in amps
   */
  public double getLeftIndexerCurrent() {
    return m_leftIndexerMotor.getOutputCurrent();
  }

  /**
   * Get right indexer motor current
   * @return Current in amps
   */
  public double getRightIndexerCurrent() {
    return m_rightIndexerMotor.getOutputCurrent();
  }

  /**
   * Get middle indexer motor current
   * @return Current in amps
   */
  public double getMiddleIndexerCurrent() {
    return m_middleIndexerMotor.getOutputCurrent();
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
   * Get the current trajectory angle for testing.
   * This is the physical hardware angle (baseline 22 degrees, adjustable in 2-degree increments).
   * @return Trajectory angle in degrees
   */
  public double getTrajectoryAngle() {
    return m_trajectoryAngle;
  }

  /**
   * Get the test RPM for manual trajectory testing.
   * @return Test RPM value
   */
  public double getTestRPM() {
    return m_testRPM;
  }

  /**
   * Check if test mode is enabled.
   * When enabled, uses test RPM instead of distance-based calculation.
   * @return True if test mode is enabled
   */
  public boolean isTestModeEnabled() {
    return m_testModeEnabled;
  }

  /**
   * NOTE: setContainer() removed - PID tuning now handled by ShooterDashboard
   * Configuration is read directly from dashboard via m_dashboard.readConfiguration()
   */

  /**
   * Calculate angle offset for velocity compensation when shooting while moving.
   * This compensates for the robot's lateral velocity component.
   *
   * @return Angle offset in radians to add to target angle (positive = lead target)
   */
  public double getVelocityCompensatedAngleOffset() {
    if (!m_velocityCompensationEnabled || m_driveSubsystem == null || m_visionSubsystem == null) {
      return 0.0;
    }

    // Convert robot-relative speeds to field-relative, then clamp to reject bump spikes
    ChassisSpeeds robotSpeeds = m_driveSubsystem.getChassisSpeeds();
    Rotation2d heading = m_driveSubsystem.getPose().getRotation();
    double vxField = robotSpeeds.vxMetersPerSecond * heading.getCos() - robotSpeeds.vyMetersPerSecond * heading.getSin();
    double vyField = robotSpeeds.vxMetersPerSecond * heading.getSin() + robotSpeeds.vyMetersPerSecond * heading.getCos();
    double maxCompVelocity = 3.0;
    double vx = Math.max(-maxCompVelocity, Math.min(maxCompVelocity, vxField));
    double vy = Math.max(-maxCompVelocity, Math.min(maxCompVelocity, vyField));
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, robotSpeeds.omegaRadiansPerSecond);

    // Get speaker position based on alliance
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return 0.0;
    }

    Translation2d speakerPosition = alliance.get() == DriverStation.Alliance.Red
        ? AutoConstants.redTarget
        : AutoConstants.blueTarget;

    // Get robot position
    Translation2d robotPosition = m_driveSubsystem.getPose().getTranslation();

    // Calculate vector from robot to speaker
    double dx = speakerPosition.getX() - robotPosition.getX();
    double dy = speakerPosition.getY() - robotPosition.getY();
    double distance = Math.sqrt(dx * dx + dy * dy);

    if (distance < 0.1) {
      return 0.0; // Too close or invalid
    }

    // Estimate time of flight
    double timeOfFlight = distance / m_averageShotVelocity;

    // Calculate where robot will be when ball reaches speaker
    double futureX = robotPosition.getX() + chassisSpeeds.vxMetersPerSecond * timeOfFlight;
    double futureY = robotPosition.getY() + chassisSpeeds.vyMetersPerSecond * timeOfFlight;

    // Calculate angle from future position to speaker
    double futureDx = speakerPosition.getX() - futureX;
    double futureDy = speakerPosition.getY() - futureY;
    double futureAngle = Math.atan2(futureDy, futureDx);

    // Calculate angle from current position to speaker
    double currentAngle = Math.atan2(dy, dx);

    // Angular offset is the difference
    double angleOffset = futureAngle - currentAngle;

    return angleOffset * m_angleCompensationFactor;
  }

  /**
   * Calculate RPM for the predicted distance when the ball arrives at the speaker.
   * Uses the radial component of robot velocity (toward/away from speaker) to predict
   * where the robot will be after the ball's flight time, then looks up RPM for that distance.
   *
   * @param currentDistance Current vision-measured distance to speaker (meters)
   * @return RPM for the predicted arrival distance
   */
  public double getVelocityCompensatedRPM(double currentDistance) {
    if (!m_velocityCompensationEnabled || m_driveSubsystem == null) {
      return getRPMForDistance(currentDistance);
    }

    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return getRPMForDistance(currentDistance);
    }

    Translation2d speakerPosition = alliance.get() == DriverStation.Alliance.Red
        ? AutoConstants.redTarget
        : AutoConstants.blueTarget;

    Translation2d robotPosition = m_driveSubsystem.getPose().getTranslation();
    // Convert robot-relative speeds to field-relative, then clamp to reject bump spikes
    ChassisSpeeds robotSpeeds = m_driveSubsystem.getChassisSpeeds();
    Rotation2d heading = m_driveSubsystem.getPose().getRotation();
    double vxField = robotSpeeds.vxMetersPerSecond * heading.getCos() - robotSpeeds.vyMetersPerSecond * heading.getSin();
    double vyField = robotSpeeds.vxMetersPerSecond * heading.getSin() + robotSpeeds.vyMetersPerSecond * heading.getCos();
    double maxCompVelocity = 3.0;
    double vx = Math.max(-maxCompVelocity, Math.min(maxCompVelocity, vxField));
    double vy = Math.max(-maxCompVelocity, Math.min(maxCompVelocity, vyField));
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, robotSpeeds.omegaRadiansPerSecond);

    // Unit vector from robot toward speaker (used to extract radial velocity)
    double dx = speakerPosition.getX() - robotPosition.getX();
    double dy = speakerPosition.getY() - robotPosition.getY();
    double poseDistance = Math.sqrt(dx * dx + dy * dy);

    if (poseDistance < 0.1) {
      return getRPMForDistance(currentDistance);
    }

    // Radial velocity: positive = moving toward speaker, negative = moving away
    double radialVelocity = (chassisSpeeds.vxMetersPerSecond * dx +
                             chassisSpeeds.vyMetersPerSecond * dy) / poseDistance;

    // Predict distance at ball arrival
    double flightTime = currentDistance / m_averageShotVelocity;
    double predictedDistance = currentDistance - radialVelocity * flightTime;

    // Clamp to the bounds of the velocity table
    double minDist = m_shooterVelocityTable[0][0];
    double maxDist = m_shooterVelocityTable[m_shooterVelocityTable.length - 1][0];
    predictedDistance = Math.max(minDist, Math.min(maxDist, predictedDistance));

    return getRPMForDistance(predictedDistance);
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
   * Update motor PID gains from dashboard configuration
   * Only reconfigures motors when values actually change to avoid disrupting control loop
   *
   * BEST PRACTICE: Centralize configuration reading in one place (dashboard) rather than
   * accessing NetworkTables entries directly from multiple locations.
   */
  private void updateMotorPIDFromDashboard() {
    // Read all configuration from dashboard
    var config = m_dashboard.readConfiguration();

    // Only update shooter motors if values changed
    if (config.shooterP != m_lastShooterP || config.shooterI != m_lastShooterI ||
        config.shooterD != m_lastShooterD || config.shooterFF != m_lastShooterFF) {

      // Configure all three shooter motors with inverted direction
      SparkFlexConfig shooterConfig = new SparkFlexConfig();
      shooterConfig
        .inverted(true)
        .closedLoop
          .pid(config.shooterP, config.shooterI, config.shooterD)
          .velocityFF(config.shooterFF);

      m_leftShooterMotor.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_rightShooterMotor.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_middleShooterMotor.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      // Update cached values
      m_lastShooterP = config.shooterP;
      m_lastShooterI = config.shooterI;
      m_lastShooterD = config.shooterD;
      m_lastShooterFF = config.shooterFF;

      System.out.println("Updated shooter PID: P=" + config.shooterP + " I=" + config.shooterI + " D=" + config.shooterD + " FF=" + config.shooterFF);
    }

    // Only update indexer motors if values changed
    if (config.indexerP != m_lastIndexerP || config.indexerI != m_lastIndexerI ||
        config.indexerD != m_lastIndexerD || config.indexerFF != m_lastIndexerFF) {

      // Configure left indexer (not inverted)
      SparkFlexConfig leftIndexerConfig = new SparkFlexConfig();
      leftIndexerConfig.closedLoop
        .pid(config.indexerP, config.indexerI, config.indexerD)
        .velocityFF(config.indexerFF);
      m_leftIndexerMotor.configure(leftIndexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      // Configure right and middle indexers with inverted direction
      SparkFlexConfig invertedIndexerConfig = new SparkFlexConfig();
      invertedIndexerConfig
        .inverted(true)
        .closedLoop
          .pid(config.indexerP, config.indexerI, config.indexerD)
          .velocityFF(config.indexerFF);
      m_rightIndexerMotor.configure(invertedIndexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_middleIndexerMotor.configure(invertedIndexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      // Update cached values
      m_lastIndexerP = config.indexerP;
      m_lastIndexerI = config.indexerI;
      m_lastIndexerD = config.indexerD;
      m_lastIndexerFF = config.indexerFF;

      System.out.println("Updated indexer PID: P=" + config.indexerP + " I=" + config.indexerI + " D=" + config.indexerD + " FF=" + config.indexerFF);
    }

    // Only update floor motor if PID or current limit changed
    boolean floorPIDChanged = config.floorMotorP != m_lastFloorMotorP ||
                              config.floorMotorI != m_lastFloorMotorI ||
                              config.floorMotorD != m_lastFloorMotorD ||
                              config.floorMotorFF != m_lastFloorMotorFF;
    boolean floorCurrentLimitChanged = config.floorMotorCurrentLimit != m_lastFloorMotorCurrentLimit;

    if (floorPIDChanged || floorCurrentLimitChanged) {
      SparkFlexConfig floorConfig = new SparkFlexConfig();
      floorConfig
          .inverted(false)
          .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
          .smartCurrentLimit(config.floorMotorCurrentLimit)
          .closedLoop
            .pid(config.floorMotorP, config.floorMotorI, config.floorMotorD)
            .velocityFF(config.floorMotorFF);
      m_floorMotor.configure(floorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      // Update cached values
      m_lastFloorMotorP = config.floorMotorP;
      m_lastFloorMotorI = config.floorMotorI;
      m_lastFloorMotorD = config.floorMotorD;
      m_lastFloorMotorFF = config.floorMotorFF;
      m_lastFloorMotorCurrentLimit = config.floorMotorCurrentLimit;

      if (floorPIDChanged) {
        System.out.println("Updated floor motor PID: P=" + config.floorMotorP + " I=" + config.floorMotorI + " D=" + config.floorMotorD + " FF=" + config.floorMotorFF);
      }
      if (floorCurrentLimitChanged) {
        System.out.println("Updated floor motor current limit: " + config.floorMotorCurrentLimit + "A");
      }
    }
  }
}

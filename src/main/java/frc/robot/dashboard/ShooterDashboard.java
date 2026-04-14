// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.dashboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * ShooterDashboard - Organizes all shooter-related telemetry and configuration
 * in a single Shuffleboard tab with clean layouts.
 *
 * BEST PRACTICE: Separate dashboard/telemetry logic from subsystem business logic.
 * This makes the dashboard organization explicit and easy to modify without
 * touching subsystem code.
 *
 * Organization:
 * - Motors layout: Real-time velocity and current for all motors
 * - PID Tuning layout: Live-tunable PID values for shooter and indexer
 * - Velocity Table layout: Distance-to-RPM lookup table tuning
 * - Testing layout: Test mode controls and trajectory testing
 * - Status layout: High-level shooter state information
 */
public class ShooterDashboard {
  // Shuffleboard tab and layouts
  private ShuffleboardTab shooterTab;
  private ShuffleboardLayout motorsLayout;
  private ShuffleboardLayout pidTuningLayout;
  private ShuffleboardLayout velocityTableLayout;
  private ShuffleboardLayout testingLayout;
  private ShuffleboardLayout statusLayout;

  // Telemetry entries (display-only values)
  private GenericEntry leftShooterVelocityEntry;
  private GenericEntry rightShooterVelocityEntry;
  private GenericEntry middleShooterVelocityEntry;
  private GenericEntry leftShooterCurrentEntry;
  private GenericEntry rightShooterCurrentEntry;
  private GenericEntry middleShooterCurrentEntry;

  private GenericEntry floorMotorVelocityEntry;
  private GenericEntry floorMotorCurrentEntry;
  private GenericEntry floorMotorSetpointEntry;

  private GenericEntry leftIndexerVelocityEntry;
  private GenericEntry rightIndexerVelocityEntry;
  private GenericEntry middleIndexerVelocityEntry;
  private GenericEntry leftIndexerCurrentEntry;
  private GenericEntry rightIndexerCurrentEntry;
  private GenericEntry middleIndexerCurrentEntry;

  private GenericEntry targetRPMEntry;
  private GenericEntry currentRPMEntry;
  private GenericEntry distanceToTargetEntry;

  private GenericEntry shooterActiveEntry;
  private GenericEntry atTargetVelocityEntry;

  // Configuration entries (user-editable values)
  private GenericEntry shooterPEntry;
  private GenericEntry shooterIEntry;
  private GenericEntry shooterDEntry;
  private GenericEntry shooterFFEntry;

  private GenericEntry indexerPEntry;
  private GenericEntry indexerIEntry;
  private GenericEntry indexerDEntry;
  private GenericEntry indexerFFEntry;

  private GenericEntry floorMotorPEntry;
  private GenericEntry floorMotorIEntry;
  private GenericEntry floorMotorDEntry;
  private GenericEntry floorMotorFFEntry;

  private GenericEntry floorMotorCurrentLimitEntry;

  // Velocity table entries (6 rows of distance/RPM pairs)
  private GenericEntry[] velocityTableDistanceEntries;
  private GenericEntry[] velocityTableRPMEntries;
  private GenericEntry preSpinRPMCapEntry;
  private GenericEntry velocityCompensationEnabledEntry;

  // Testing entries
  private GenericEntry testModeEnabledEntry;
  private GenericEntry testRPMEntry;
  private GenericEntry trajectoryAngleEntry;

  /**
   * Configuration data class for passing config values back to subsystem.
   * BEST PRACTICE: Use data classes to group related configuration values
   * rather than passing many individual parameters.
   */
  public static class ShooterConfig {
    public double shooterP;
    public double shooterI;
    public double shooterD;
    public double shooterFF;

    public double indexerP;
    public double indexerI;
    public double indexerD;
    public double indexerFF;

    public double floorMotorP;
    public double floorMotorI;
    public double floorMotorD;
    public double floorMotorFF;

    public int floorMotorCurrentLimit;

    public double[][] velocityTable;
    public double preSpinRPMCap;
    public boolean velocityCompensationEnabled;
    public double angleCompensationFactor;
    public double averageShotVelocity;

    public boolean testModeEnabled;
    public double testRPM;
    public double trajectoryAngle;
  }

  /**
   * Initialize the Shooter dashboard tab and all layouts.
   * BEST PRACTICE: Create all Shuffleboard widgets during initialization,
   * not during periodic updates. This prevents UI flickering and performance issues.
   */
  public void initialize() {
    // Create main tab
    shooterTab = Shuffleboard.getTab("Shooter");

    // Initialize all layouts
    initializeMotorsLayout();
    initializePIDTuningLayout();
    initializeVelocityTableLayout();
    initializeTestingLayout();
    initializeStatusLayout();
  }

  /**
   * Initialize the Motors layout with velocity and current telemetry.
   */
  private void initializeMotorsLayout() {
    motorsLayout = shooterTab.getLayout("Motors", "List Layout")
        .withSize(2, 4)
        .withPosition(0, 0);

    // Shooter motor telemetry
    leftShooterVelocityEntry = motorsLayout.add("Left Shooter Velocity", 0.0).getEntry();
    rightShooterVelocityEntry = motorsLayout.add("Right Shooter Velocity", 0.0).getEntry();
    middleShooterVelocityEntry = motorsLayout.add("Middle Shooter Velocity", 0.0).getEntry();

    leftShooterCurrentEntry = motorsLayout.add("Left Shooter Current", 0.0).getEntry();
    rightShooterCurrentEntry = motorsLayout.add("Right Shooter Current", 0.0).getEntry();
    middleShooterCurrentEntry = motorsLayout.add("Middle Shooter Current", 0.0).getEntry();

    // Floor motor telemetry
    floorMotorVelocityEntry = motorsLayout.add("Floor Motor Velocity", 0.0).getEntry();
    floorMotorCurrentEntry = motorsLayout.add("Floor Motor Current", 0.0).getEntry();
    floorMotorSetpointEntry = motorsLayout.add("Floor Motor Setpoint", 0.0).getEntry();

    // Indexer motor telemetry
    leftIndexerVelocityEntry = motorsLayout.add("Left Indexer Velocity", 0.0).getEntry();
    rightIndexerVelocityEntry = motorsLayout.add("Right Indexer Velocity", 0.0).getEntry();
    middleIndexerVelocityEntry = motorsLayout.add("Middle Indexer Velocity", 0.0).getEntry();

    leftIndexerCurrentEntry = motorsLayout.add("Left Indexer Current", 0.0).getEntry();
    rightIndexerCurrentEntry = motorsLayout.add("Right Indexer Current", 0.0).getEntry();
    middleIndexerCurrentEntry = motorsLayout.add("Middle Indexer Current", 0.0).getEntry();
  }

  /**
   * Initialize the PID Tuning layout with editable PID values.
   * BEST PRACTICE: Use text view widgets for numeric inputs to allow live tuning.
   */
  private void initializePIDTuningLayout() {
    pidTuningLayout = shooterTab.getLayout("PID Tuning", "List Layout")
        .withSize(2, 4)
        .withPosition(2, 0);

    // Shooter PID
    shooterPEntry = pidTuningLayout.add("Shooter P", ShooterConstants.kShooterP)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    shooterIEntry = pidTuningLayout.add("Shooter I", ShooterConstants.kShooterI)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    shooterDEntry = pidTuningLayout.add("Shooter D", ShooterConstants.kShooterD)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    shooterFFEntry = pidTuningLayout.add("Shooter FF", ShooterConstants.kShooterFF)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    // Indexer PID
    indexerPEntry = pidTuningLayout.add("Indexer P", ShooterConstants.kIndexerP)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    indexerIEntry = pidTuningLayout.add("Indexer I", ShooterConstants.kIndexerI)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    indexerDEntry = pidTuningLayout.add("Indexer D", ShooterConstants.kIndexerD)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    indexerFFEntry = pidTuningLayout.add("Indexer FF", ShooterConstants.kIndexerFF)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    // Floor motor PID
    floorMotorPEntry = pidTuningLayout.add("Floor Motor P", ShooterConstants.kFloorMotorP)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    floorMotorIEntry = pidTuningLayout.add("Floor Motor I", ShooterConstants.kFloorMotorI)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    floorMotorDEntry = pidTuningLayout.add("Floor Motor D", ShooterConstants.kFloorMotorD)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    floorMotorFFEntry = pidTuningLayout.add("Floor Motor FF", ShooterConstants.kFloorMotorFF)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    // Floor motor current limit
    floorMotorCurrentLimitEntry = pidTuningLayout.add("Floor Motor Current Limit", ShooterConstants.kFloorMotorCurrentLimit)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
  }

  /**
   * Initialize the Velocity Table layout for distance-to-RPM tuning.
   */
  private void initializeVelocityTableLayout() {
    velocityTableLayout = shooterTab.getLayout("Velocity Table", "List Layout")
        .withSize(2, 4)
        .withPosition(4, 0);

    // Create entries for each row of the velocity table
    velocityTableDistanceEntries = new GenericEntry[6];
    velocityTableRPMEntries = new GenericEntry[6];

    for (int i = 0; i < 6; i++) {
      velocityTableDistanceEntries[i] = velocityTableLayout
          .add("Row " + i + " Distance (m)", ShooterConstants.kShooterVelocityTable[i][0])
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();

      velocityTableRPMEntries[i] = velocityTableLayout
          .add("Row " + i + " RPM", ShooterConstants.kShooterVelocityTable[i][1])
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
    }

    // Pre-spin RPM cap
    preSpinRPMCapEntry = velocityTableLayout.add("Pre-Spin RPM Cap", ShooterConstants.kPreSpinRPMCap)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    // Velocity compensation toggle
    velocityCompensationEnabledEntry = velocityTableLayout
        .add("Velocity Compensation", ShooterConstants.kVelocityCompensationEnabled)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
  }

  /**
   * Initialize the Testing layout for test mode and trajectory testing.
   */
  private void initializeTestingLayout() {
    testingLayout = shooterTab.getLayout("Testing", "List Layout")
        .withSize(2, 3)
        .withPosition(6, 0);

    testModeEnabledEntry = testingLayout.add("Test Mode Enabled", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();

    testRPMEntry = testingLayout.add("Test RPM", 3000.0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    trajectoryAngleEntry = testingLayout.add("Trajectory Angle", 22.0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    distanceToTargetEntry = testingLayout.add("Distance to Target", 0.0).getEntry();
  }

  /**
   * Initialize the Status layout for high-level shooter state.
   */
  private void initializeStatusLayout() {
    statusLayout = shooterTab.getLayout("Status", "List Layout")
        .withSize(2, 3)
        .withPosition(8, 0);

    targetRPMEntry = statusLayout.add("Target RPM", 0.0).getEntry();

    currentRPMEntry = statusLayout.add("Current RPM", 0.0).getEntry();

    shooterActiveEntry = statusLayout.add("Shooter Active", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    atTargetVelocityEntry = statusLayout.add("At Target Velocity", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();
  }

  /**
   * Update all telemetry values from the shooter subsystem.
   * Call this from ShooterSubsystem.periodic()
   *
   * BEST PRACTICE: Throttle telemetry updates to avoid CAN bus saturation.
   * Only update every N periodic cycles (handled by subsystem).
   *
   * @param shooter The shooter subsystem to read values from
   */
  public void updateTelemetry(ShooterSubsystem shooter) {
    // Motor velocities
    leftShooterVelocityEntry.setDouble(shooter.getLeftShooterVelocity());
    rightShooterVelocityEntry.setDouble(shooter.getRightShooterVelocity());
    middleShooterVelocityEntry.setDouble(shooter.getMiddleShooterVelocity());

    // Motor currents
    leftShooterCurrentEntry.setDouble(shooter.getLeftShooterCurrent());
    rightShooterCurrentEntry.setDouble(shooter.getRightShooterCurrent());
    middleShooterCurrentEntry.setDouble(shooter.getMiddleShooterCurrent());

    // Floor motor
    floorMotorVelocityEntry.setDouble(shooter.getFloorMotorVelocity());
    floorMotorCurrentEntry.setDouble(shooter.getFloorMotorCurrent());
    floorMotorSetpointEntry.setDouble(shooter.getFloorMotorSetpoint());

    // Indexer velocities
    leftIndexerVelocityEntry.setDouble(shooter.getLeftIndexerVelocity());
    rightIndexerVelocityEntry.setDouble(shooter.getRightIndexerVelocity());
    middleIndexerVelocityEntry.setDouble(shooter.getMiddleIndexerVelocity());

    // Indexer currents
    leftIndexerCurrentEntry.setDouble(shooter.getLeftIndexerCurrent());
    rightIndexerCurrentEntry.setDouble(shooter.getRightIndexerCurrent());
    middleIndexerCurrentEntry.setDouble(shooter.getMiddleIndexerCurrent());

    // Status
    targetRPMEntry.setDouble(shooter.getCurrentTargetRPM());
    currentRPMEntry.setDouble(shooter.getAverageShooterRPM());
    shooterActiveEntry.setBoolean(shooter.isShooterActive());
    atTargetVelocityEntry.setBoolean(shooter.isAtTargetVelocity(100.0));
    distanceToTargetEntry.setDouble(shooter.getDistanceToSpeaker());
  }

  /**
   * Read configuration values from dashboard and return as a config object.
   * Call this from ShooterSubsystem.periodic() to get live-tuned values.
   *
   * BEST PRACTICE: Return a configuration object rather than having the
   * subsystem directly access dashboard entries. This maintains encapsulation
   * and makes testing easier.
   *
   * @return ShooterConfig object with current dashboard values
   */
  public ShooterConfig readConfiguration() {
    ShooterConfig config = new ShooterConfig();

    // PID values
    config.shooterP = shooterPEntry.getDouble(ShooterConstants.kShooterP);
    config.shooterI = shooterIEntry.getDouble(ShooterConstants.kShooterI);
    config.shooterD = shooterDEntry.getDouble(ShooterConstants.kShooterD);
    config.shooterFF = shooterFFEntry.getDouble(ShooterConstants.kShooterFF);

    config.indexerP = indexerPEntry.getDouble(ShooterConstants.kIndexerP);
    config.indexerI = indexerIEntry.getDouble(ShooterConstants.kIndexerI);
    config.indexerD = indexerDEntry.getDouble(ShooterConstants.kIndexerD);
    config.indexerFF = indexerFFEntry.getDouble(ShooterConstants.kIndexerFF);

    config.floorMotorP = floorMotorPEntry.getDouble(ShooterConstants.kFloorMotorP);
    config.floorMotorI = floorMotorIEntry.getDouble(ShooterConstants.kFloorMotorI);
    config.floorMotorD = floorMotorDEntry.getDouble(ShooterConstants.kFloorMotorD);
    config.floorMotorFF = floorMotorFFEntry.getDouble(ShooterConstants.kFloorMotorFF);

    config.floorMotorCurrentLimit = (int) floorMotorCurrentLimitEntry.getDouble(ShooterConstants.kFloorMotorCurrentLimit);

    // Velocity table
    config.velocityTable = new double[6][2];
    for (int i = 0; i < 6; i++) {
      config.velocityTable[i][0] = velocityTableDistanceEntries[i].getDouble(
          ShooterConstants.kShooterVelocityTable[i][0]);
      config.velocityTable[i][1] = velocityTableRPMEntries[i].getDouble(
          ShooterConstants.kShooterVelocityTable[i][1]);
    }

    config.preSpinRPMCap = preSpinRPMCapEntry.getDouble(ShooterConstants.kPreSpinRPMCap);
    config.velocityCompensationEnabled = velocityCompensationEnabledEntry.getBoolean(
        ShooterConstants.kVelocityCompensationEnabled);
    config.angleCompensationFactor = ShooterConstants.kAngleCompensationFactor;  // TODO: Add to dashboard if needed
    config.averageShotVelocity = ShooterConstants.kAverageShotVelocity;  // TODO: Add to dashboard if needed

    // Testing
    config.testModeEnabled = testModeEnabledEntry.getBoolean(false);
    config.testRPM = testRPMEntry.getDouble(3000.0);
    config.trajectoryAngle = trajectoryAngleEntry.getDouble(22.0);

    return config;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.dashboard;

/**
 * DashboardManager - Singleton coordinator for all dashboard sections.
 *
 * BEST PRACTICE: Centralize dashboard management in a single class rather than
 * scattering NetworkTables/Shuffleboard setup across subsystems. This makes it
 * easier to organize, maintain, and find values during competition.
 *
 * Each subsystem gets its own dashboard class (e.g., ShooterDashboard) that
 * handles telemetry publishing and configuration reading for that subsystem.
 */
public class DashboardManager {
  private static DashboardManager instance;

  private final ShooterDashboard shooterDashboard;
  // Add more dashboard instances as needed:
  // private final CollectorDashboard collectorDashboard;
  // private final DriveDashboard driveDashboard;
  // private final VisionDashboard visionDashboard;

  /**
   * Private constructor for singleton pattern.
   * BEST PRACTICE: Use singleton for global managers that should only have one instance.
   */
  private DashboardManager() {
    shooterDashboard = new ShooterDashboard();
    // Initialize other dashboards here when created
  }

  /**
   * Get the singleton instance of DashboardManager.
   * @return The single DashboardManager instance
   */
  public static DashboardManager getInstance() {
    if (instance == null) {
      instance = new DashboardManager();
    }
    return instance;
  }

  /**
   * Initialize all dashboard sections.
   * Call this once from RobotContainer constructor.
   *
   * BEST PRACTICE: Separate initialization from construction to control when
   * Shuffleboard tabs are created and allow proper ordering.
   */
  public void initialize() {
    shooterDashboard.initialize();
    // Initialize other dashboards here when created
  }

  /**
   * Get the shooter dashboard instance.
   * @return ShooterDashboard instance
   */
  public ShooterDashboard getShooterDashboard() {
    return shooterDashboard;
  }

  // Add getters for other dashboards as needed:
  // public CollectorDashboard getCollectorDashboard() { return collectorDashboard; }
  // public DriveDashboard getDriveDashboard() { return driveDashboard; }
  // public VisionDashboard getVisionDashboard() { return visionDashboard; }
}

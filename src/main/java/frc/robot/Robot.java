// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  @SuppressWarnings("unused")
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // Enable 100% vision trust while disabled to maintain accurate field-relative pose
    // This allows moving the robot while disabled and corrects field orientation
    m_robotContainer.getVisionSubsystem().enableVisionPoseReset();
  }

  @Override
  public void disabledPeriodic() {
    // Update SmartDashboard with autonomous selection based on current pose
    // This updates continuously (every 20ms) while disabled so drivers can see
    // which autonomous will run based on robot position
    m_robotContainer.updateAutoDisplay();
  }

  @Override
  public void autonomousInit() {
    // Switch to sensor fusion mode (vision + odometry)
    m_robotContainer.getVisionSubsystem().disableVisionPoseReset();

    // Start shooter spinning at the beginning of autonomous
    m_robotContainer.getShooterSubsystem().activateShooter();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Switch to sensor fusion mode (vision + odometry)
    m_robotContainer.getVisionSubsystem().disableVisionPoseReset();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Spin up shooter on teleop enable (uses current before driving starts)
    m_robotContainer.spinUpShooter();

    // Deploy hopper to down position on teleop enable
    m_robotContainer.getCollector().extendHopper();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

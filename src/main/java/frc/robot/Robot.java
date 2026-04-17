// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.led.LEDSubsystem.LEDMode;

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
    System.out.println(">>> DISABLED INIT CALLED <<<");

    // Set LED mode to DISABLED (show auto selection pattern)
    m_robotContainer.getLEDSubsystem().setMode(LEDMode.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    // Update SmartDashboard with autonomous selection based on current pose
    // This updates continuously (every 20ms) while disabled so drivers can see
    // which autonomous will run based on robot position
    m_robotContainer.updateAutoDisplay();

    // Update LED auto position based on current pose
    var pose = m_robotContainer.getSwerveDrive().getPose();
    m_robotContainer.getLEDSubsystem().setAutoPositionFromPose(pose.getX(), pose.getY());

    // Check if AprilTags are visible
    int activeCameras = m_robotContainer.getVisionSubsystem().getActiveCameraCount();
    if (activeCameras == 0) {
      // No AprilTags detected - flash yellow
      m_robotContainer.getLEDSubsystem().setMode(LEDMode.NO_APRILTAGS);
    } else {
      // AprilTags detected - show auto selection
      m_robotContainer.getLEDSubsystem().setMode(LEDMode.DISABLED);
    }
  }

  @Override
  public void autonomousInit() {
    // Switch to sensor fusion mode (vision + odometry)
    m_robotContainer.getVisionSubsystem().disableVisionPoseReset();

    // REMOVED: Shooter now starts on-demand when trigger pressed (staggered startup in auto commands)
    // m_robotContainer.getShooterSubsystem().activateShooter();

    // Set LED mode to AUTONOMOUS (flash alliance color)
    m_robotContainer.getLEDSubsystem().setMode(LEDMode.AUTONOMOUS);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // DEBUG: Log which command was selected (both console and SmartDashboard)
    System.out.println("=== AUTONOMOUS INIT ===");
    System.out.println("Selected command: " + (m_autonomousCommand != null ? m_autonomousCommand.getClass().getSimpleName() : "NULL"));
    System.out.println("Robot pose: " + m_robotContainer.getSwerveDrive().getPose());

    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Debug/AutoCommand",
        m_autonomousCommand != null ? m_autonomousCommand.getClass().getSimpleName() : "NULL");
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Debug/AutoPose",
        m_robotContainer.getSwerveDrive().getPose().toString());

    // Schedule the autonomous command
    if (m_autonomousCommand != null) {
      System.out.println("Scheduling autonomous command...");
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Debug/SchedulingCommand", true);
      m_autonomousCommand.schedule();
      System.out.println("Command scheduled!");
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Debug/CommandScheduled", true);
    } else {
      System.out.println("ERROR: Autonomous command is NULL!");
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Debug/CommandNULL", true);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    System.out.println(">>> TELEOP INIT CALLED <<<");

    // Switch to sensor fusion mode (vision + odometry)
    m_robotContainer.getVisionSubsystem().disableVisionPoseReset();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // REMOVED: Shooter now starts on-demand when trigger pressed
    // m_robotContainer.spinUpShooter();

    // Deploy hopper to down position on teleop enable
    m_robotContainer.getCollector().extendHopper();

    // Set LED mode to TELEOP (show hub status and warnings)
    m_robotContainer.getLEDSubsystem().setMode(LEDMode.TELEOP);

    // Get hub game data from FMS
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData != null && gameData.length() > 0) {
      char hubData = gameData.charAt(0);
      m_robotContainer.getLEDSubsystem().setHubGameData(hubData);
      System.out.println("Hub game data: " + hubData);
    } else {
      System.out.println("No hub game data available");
    }
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

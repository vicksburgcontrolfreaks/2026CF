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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // Use vision to align field-oriented drive with actual field orientation
    // This allows the robot to start in any orientation and still have accurate field-oriented control
    edu.wpi.first.math.geometry.Rotation2d visionRotation =
        m_robotContainer.getVisionSubsystem().getVisionRotationEstimate();

    if (visionRotation != null) {
      // Vision has a valid rotation estimate - use it to set the gyro heading
      m_robotContainer.getSwerveDrive().setHeading(visionRotation);
      System.out.println("Gyro aligned to field using vision: " + visionRotation.getDegrees() + " degrees");
    } else {
      // No vision estimate available - warn the user
      System.out.println("WARNING: No AprilTags visible - field orientation will be relative to robot starting position");
    }

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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
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

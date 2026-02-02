// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Autonomous command to drive the robot forward a specified distance.
 * Uses odometry to track distance traveled.
 */
public class DriveForwardCommand extends Command {
  private final SwerveDrive m_swerveDrive;
  private final double m_targetDistance; // meters
  private final double m_speed; // m/s
  private Pose2d m_startPose;

  /**
   * Creates a new DriveForwardCommand.
   *
   * @param swerveDrive The swerve drive subsystem
   * @param distanceMeters Distance to drive forward in meters
   * @param speedMetersPerSecond Speed to drive at in m/s
   */
  public DriveForwardCommand(SwerveDrive swerveDrive, double distanceMeters, double speedMetersPerSecond) {
    m_swerveDrive = swerveDrive;
    m_targetDistance = distanceMeters;
    m_speed = speedMetersPerSecond;

    addRequirements(swerveDrive);
  }

  /**
   * Creates a new DriveForwardCommand with default speed of 0.5 m/s.
   *
   * @param swerveDrive The swerve drive subsystem
   * @param distanceMeters Distance to drive forward in meters
   */
  public DriveForwardCommand(SwerveDrive swerveDrive, double distanceMeters) {
    this(swerveDrive, distanceMeters, 0.5); // Default 0.5 m/s
  }

  @Override
  public void initialize() {
    // Record starting position
    m_startPose = m_swerveDrive.getPose();
    System.out.println("DriveForwardCommand starting at: " + m_startPose);
  }

  @Override
  public void execute() {
    // Drive forward at the specified speed
    // xSpeed is forward/backward (positive = forward)
    // ySpeed is left/right (0 = straight)
    // rot is rotation (0 = no rotation)
    // fieldRelative = false (robot-relative movement)
    m_swerveDrive.drive(m_speed, 0, 0, false);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    m_swerveDrive.stop();

    Pose2d endPose = m_swerveDrive.getPose();
    double distanceTraveled = m_startPose.getTranslation().getDistance(endPose.getTranslation());

    System.out.println("DriveForwardCommand ended at: " + endPose);
    System.out.println("Distance traveled: " + distanceTraveled + " meters");

    if (interrupted) {
      System.out.println("DriveForwardCommand was interrupted!");
    }
  }

  @Override
  public boolean isFinished() {
    // Get current position
    Pose2d currentPose = m_swerveDrive.getPose();

    // Calculate distance traveled from start
    double distanceTraveled = m_startPose.getTranslation().getDistance(currentPose.getTranslation());

    // Finish when we've traveled the target distance
    return distanceTraveled >= m_targetDistance;
  }
}

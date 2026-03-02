// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Command that rotates the robot to face a target position on the field using PID control.
 * The robot will calculate the angle to the target and rotate to face it.
 */
public class RotateToTargetCommand extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final Translation2d m_targetPosition;
  private final PIDController m_rotationController;

  /**
   * Creates a new RotateToTargetCommand.
   *
   * @param swerveDrive The swerve drive subsystem
   * @param targetPosition The target position on the field (X, Y in meters)
   */
  public RotateToTargetCommand(SwerveDriveSubsystem swerveDrive, Translation2d targetPosition) {
    m_swerveDrive = swerveDrive;
    m_targetPosition = targetPosition;

    m_rotationController = new PIDController(
        AutoConstants.kRotateToTargetP,
        AutoConstants.kRotateToTargetI,
        AutoConstants.kRotateToTargetD
    );

    // Enable continuous input for angle wrapping (-180 to 180 degrees)
    m_rotationController.enableContinuousInput(-180, 180);
    m_rotationController.setTolerance(AutoConstants.kRotateToTargetTolerance);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    // Reset the PID controller
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    // Get current robot pose
    Pose2d currentPose = m_swerveDrive.getPose();

    // Calculate the angle to the target
    double deltaX = m_targetPosition.getX() - currentPose.getX();
    double deltaY = m_targetPosition.getY() - currentPose.getY();
    double targetAngleDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 180.0;

    // Get current heading
    double currentHeading = m_swerveDrive.getHeading();

    // Calculate rotation using PID
    double rotationSpeed = m_rotationController.calculate(currentHeading, targetAngleDegrees);

    // Clamp rotation speed to max velocity
    rotationSpeed = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
                             Math.min(AutoConstants.kRotateToTargetMaxVelocity, rotationSpeed));

    // Apply rotation to drive (no translation)
    m_swerveDrive.drive(
        0.0,
        0.0,
        rotationSpeed * SwerveConstants.kMaxAngularSpeedRadiansPerSecond,
        false
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return m_rotationController.atSetpoint();
  }
}

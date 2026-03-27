// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Command that allows driver to control translation (forward/backward, left/right)
 * while automatically maintaining rotation to face the back of the robot toward a target.
 *
 * This enables "shooting while moving" - the driver controls where the robot goes,
 * but the robot automatically keeps aligned to the target for accurate shots.
 */
public class DriveAndAlignCommand extends Command {
  private final DriveSubsystem m_swerveDrive;
  private final Translation2d m_targetPosition;
  private final PIDController m_rotationController;

  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;

  /**
   * Creates a new DriveAndAlignCommand.
   *
   * @param swerveDrive The swerve drive subsystem
   * @param targetPosition The target position on the field to face (X, Y in meters)
   * @param xSpeedSupplier Supplier for forward/backward speed (driver input)
   * @param ySpeedSupplier Supplier for left/right speed (driver input)
   */
  public DriveAndAlignCommand(
      DriveSubsystem swerveDrive,
      Translation2d targetPosition,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier) {
    m_swerveDrive = swerveDrive;
    m_targetPosition = targetPosition;
    m_xSpeedSupplier = xSpeedSupplier;
    m_ySpeedSupplier = ySpeedSupplier;

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
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    // Get current robot pose
    Pose2d currentPose = m_swerveDrive.getPose();

    // Calculate the angle to the target
    double deltaX = m_targetPosition.getX() - currentPose.getX();
    double deltaY = m_targetPosition.getY() - currentPose.getY();
    double targetAngleDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));

    // Add 180 degrees to point the BACK of the robot at the target
    targetAngleDegrees = (targetAngleDegrees + 180) % 360;
    if (targetAngleDegrees > 180) {
      targetAngleDegrees -= 360;
    }

    // Get current heading
    double currentHeading = m_swerveDrive.getHeading();

    // Calculate rotation using PID
    double rotationSpeed = m_rotationController.calculate(currentHeading, targetAngleDegrees);

    // Clamp rotation speed to max velocity
    rotationSpeed = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
                             Math.min(AutoConstants.kRotateToTargetMaxVelocity, rotationSpeed));

    // Get driver translation inputs
    double xSpeed = m_xSpeedSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeed = m_ySpeedSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond;

    // Apply driver translation + automatic rotation
    m_swerveDrive.drive(
        xSpeed,
        ySpeed,
        rotationSpeed * DriveConstants.kMaxAngularSpeed,
        true  // field-relative
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    // Never finishes - runs until interrupted
    return false;
  }

  /**
   * Check if robot is aligned to target within tolerance
   * @return True if aligned within tolerance
   */
  public boolean isAligned() {
    return m_rotationController.atSetpoint();
  }
}

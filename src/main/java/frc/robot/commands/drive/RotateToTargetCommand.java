// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Command to rotate the robot to face a specific target position on the field.
 * Calculates the required angle based on the robot's current position and the target position.
 */
public class RotateToTargetCommand extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final Translation2d m_targetPosition;
  private final PIDController m_pidController;
  private double m_targetAngle;
  private double m_slowdownThreshold;
  private double m_extremelyCloseThreshold;
  private int m_onTargetCount;

  // Tolerance for considering the rotation complete (in degrees)
  private static final double kAngleToleranceDegrees = 0.5;

  // Number of consecutive cycles we must be on target before finishing
  private static final int kOnTargetCyclesRequired = 3;

  // Maximum rotation speed (radians per second) when far from target
  private static final double kMaxRotationSpeed = 4.0;

  // Minimum rotation speed (radians per second) when close to target (max speed / 3)
  private static final double kMinRotationSpeed = kMaxRotationSpeed / 3.0;

  // Extremely slow rotation speed for final precision (max speed / 6)
  private static final double kExtremelySlowSpeed = kMaxRotationSpeed / 6.0;

  // Base angle threshold (degrees) where smoothing starts to slow down
  private static final double kBaseSlowdownAngleThreshold = 20.0;

  // Base angle threshold (degrees) for extremely slow final approach
  private static final double kBaseExtremelyCloseThreshold = 5.0;

  // Threshold to determine if initial rotation is "small" (40 degrees or less)
  private static final double kSmallRotationThreshold = 40.0;

  /**022
   * Creates a new RotateToTargetCommand.
   *
   * @param swerveDrive The swerve drive subsystem
   * @param targetX The target X coordinate in meters
   * @param targetY The target Y coordinate in meters
   */
  public RotateToTargetCommand(SwerveDriveSubsystem swerveDrive, double targetX, double targetY) {
    m_swerveDrive = swerveDrive;
    m_targetPosition = new Translation2d(targetX, targetY);

    // Create PID controller with auto-align specific rotation constants
    m_pidController = new PIDController(
      AutoConstants.kPRotationAutoAlign,
      AutoConstants.kIRotationAutoAlign,
      AutoConstants.kDRotationAutoAlign
    );

    // Enable continuous input for angle wrapping (-180 to 180 degrees)
    m_pidController.enableContinuousInput(-180.0, 180.0);

    // Set tolerance for when we consider the command finished
    m_pidController.setTolerance(kAngleToleranceDegrees);

    addRequirements(m_swerveDrive);
  }

  @Override
  public void initialize() {
    // Get current robot pose
    Pose2d currentPose = m_swerveDrive.getPose();
    Translation2d currentPosition = currentPose.getTranslation();

    // Calculate the vector from robot to target
    Translation2d toTarget = m_targetPosition.minus(currentPosition);

    // Calculate the angle to face the target with the FRONT of the robot
    // atan2 returns angle in radians, convert to degrees
    // In FRC coordinates: 0° = facing +X (toward red alliance wall)
    // 90° = facing +Y (toward left when looking from red alliance)
    // NOTE: Add 180 degrees because the robot's physical front is opposite to the pose orientation
    m_targetAngle = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX())) + 180.0;

    // Calculate initial angle error to determine appropriate smoothing ranges
    double currentHeading = m_swerveDrive.getHeading();
    double initialAngleError = Math.abs(m_targetAngle - currentHeading);
    if (initialAngleError > 180.0) {
      initialAngleError = 360.0 - initialAngleError;
    }

    // Apply different multipliers based on initial rotation magnitude
    // Small rotations (≤40°): 1.45x multiplier for more gradual slowdown
    // Large rotations (>40°): 1.75x multiplier for even more gradual slowdown
    double multiplier = (initialAngleError <= kSmallRotationThreshold) ? 1.45 : 1.75;
    m_slowdownThreshold = kBaseSlowdownAngleThreshold * multiplier;
    m_extremelyCloseThreshold = kBaseExtremelyCloseThreshold * multiplier;

    // Reset the on-target counter and PID controller
    m_onTargetCount = 0;
    m_pidController.reset();

    // Set the setpoint to the calculated target angle
    m_pidController.setSetpoint(m_targetAngle);

    System.out.printf("RotateToTarget: Robot at (%.2f, %.2f), Target at (%.2f, %.2f), Rotating to %.2f degrees (initial error: %.1f°, slowdown: %.1f°, extremely close: %.1f°)%n",
      currentPosition.getX(), currentPosition.getY(),
      m_targetPosition.getX(), m_targetPosition.getY(),
      m_targetAngle, initialAngleError, m_slowdownThreshold, m_extremelyCloseThreshold);
  }

  @Override
  public void execute() {
    // Get current heading
    double currentHeading = m_swerveDrive.getHeading();

    // Calculate angle error (how far we are from target)
    double angleError = Math.abs(m_targetAngle - currentHeading);
    // Handle angle wrapping (e.g., error between 350° and 10° should be 20°, not 340°)
    if (angleError > 180.0) {
      angleError = 360.0 - angleError;
    }

    // Calculate dynamic max speed based on angle error (three-tier smoothing with adaptive thresholds)
    // Far: angleError > slowdown threshold → use max speed (4.0 rad/s)
    // Close: extremely close threshold < angleError < slowdown threshold → interpolate (1.33 to 4.0 rad/s)
    // Extremely close: angleError < extremely close threshold → use extremely slow speed (0.67 rad/s)
    double dynamicMaxSpeed;
    if (angleError >= m_slowdownThreshold) {
      // Far from target - use maximum speed
      dynamicMaxSpeed = kMaxRotationSpeed;
    } else if (angleError >= m_extremelyCloseThreshold) {
      // Close to target - interpolate between min and max speed
      double speedRange = kMaxRotationSpeed - kMinRotationSpeed;
      double errorRatio = (angleError - m_extremelyCloseThreshold) / (m_slowdownThreshold - m_extremelyCloseThreshold);
      dynamicMaxSpeed = kMinRotationSpeed + (speedRange * errorRatio);
    } else {
      // Extremely close to target - use extremely slow speed for final precision
      dynamicMaxSpeed = kExtremelySlowSpeed;
    }

    // Calculate rotation speed using PID controller
    double rotationSpeed = m_pidController.calculate(currentHeading);

    // Clamp the rotation speed to dynamic maximum
    rotationSpeed = MathUtil.clamp(rotationSpeed, -dynamicMaxSpeed, dynamicMaxSpeed);

    // Track if we're on target for settling
    if (m_pidController.atSetpoint()) {
      m_onTargetCount++;
    } else {
      m_onTargetCount = 0;
    }

    // Drive with only rotation (no translation)
    m_swerveDrive.drive(0, 0, rotationSpeed, false);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      // If interrupted, just stop the robot normally
      m_swerveDrive.stop();
      System.out.println("RotateToTarget: Command interrupted");
    } else {
      // Successfully aligned - lock wheels in X-formation for stability
      m_swerveDrive.setX();
      System.out.printf("RotateToTarget: Aligned to target at %.2f degrees - Wheels locked%n", m_targetAngle);
    }
  }

  @Override
  public boolean isFinished() {
    // Command is finished when we're within tolerance for multiple consecutive cycles
    // This ensures the robot has settled and isn't just passing through the target
    return m_onTargetCount >= kOnTargetCyclesRequired;
  }
}

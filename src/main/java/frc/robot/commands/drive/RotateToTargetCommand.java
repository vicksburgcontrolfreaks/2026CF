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

  // Tolerance for considering the rotation complete (in degrees)
  private static final double kAngleToleranceDegrees = 3.0;

  // Maximum rotation speed (radians per second) - slower for precise target alignment
  private static final double kMaxRotationSpeed = 0.75;

  /**
   * Creates a new RotateToTargetCommand.
   *
   * @param swerveDrive The swerve drive subsystem
   * @param targetX The target X coordinate in meters
   * @param targetY The target Y coordinate in meters
   */
  public RotateToTargetCommand(SwerveDriveSubsystem swerveDrive, double targetX, double targetY) {
    m_swerveDrive = swerveDrive;
    m_targetPosition = new Translation2d(targetX, targetY);

    // Create PID controller with rotation constants from AutoConstants
    m_pidController = new PIDController(
      AutoConstants.kPRotation,
      AutoConstants.kIRotation,
      AutoConstants.kDRotation
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

    // Calculate the angle to face AWAY from the target (back of robot toward target)
    // atan2 returns angle in radians, convert to degrees
    // In FRC coordinates: 0° = facing +X (toward red alliance wall)
    // 90° = facing +Y (toward left when looking from red alliance)
    // Add 180 degrees to face backwards
    m_targetAngle = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX())) + 180.0;

    // Reset the PID controller
    m_pidController.reset();

    // Set the setpoint to the calculated target angle
    m_pidController.setSetpoint(m_targetAngle);

    System.out.printf("RotateToTarget: Robot at (%.2f, %.2f), Target at (%.2f, %.2f), Rotating to %.2f degrees%n",
      currentPosition.getX(), currentPosition.getY(),
      m_targetPosition.getX(), m_targetPosition.getY(),
      m_targetAngle);
  }

  @Override
  public void execute() {
    // Get current heading
    double currentHeading = m_swerveDrive.getHeading();

    // Calculate rotation speed using PID controller
    double rotationSpeed = m_pidController.calculate(currentHeading);

    // Clamp the rotation speed to maximum
    rotationSpeed = MathUtil.clamp(rotationSpeed, -kMaxRotationSpeed, kMaxRotationSpeed);

    // Drive with only rotation (no translation)
    m_swerveDrive.drive(0, 0, rotationSpeed, false);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    m_swerveDrive.stop();

    if (interrupted) {
      System.out.println("RotateToTarget: Command interrupted");
    } else {
      System.out.printf("RotateToTarget: Facing target at %.2f degrees%n", m_targetAngle);
    }
  }

  @Override
  public boolean isFinished() {
    // Command is finished when we're within tolerance of the target angle
    return m_pidController.atSetpoint();
  }
}

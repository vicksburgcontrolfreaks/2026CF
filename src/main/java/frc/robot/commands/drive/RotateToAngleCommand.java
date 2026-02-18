// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Command to rotate the robot to a specified angle using PID control.
 * The angle is in degrees and is field-relative (0 degrees = forward from driver perspective).
 */
public class RotateToAngleCommand extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final double m_targetAngleDegrees;
  private final PIDController m_pidController;

  // Tolerance for considering the rotation complete (in degrees)
  private static final double kAngleToleranceDegrees = 2.0;

  // Maximum rotation speed (radians per second)
  private static final double kMaxRotationSpeed = 0.5;

  /**
   * Creates a new RotateToAngleCommand.
   *
   * @param swerveDrive The swerve drive subsystem
   * @param targetAngleDegrees The target angle in degrees (0 = forward, positive = counter-clockwise)
   */
  public RotateToAngleCommand(SwerveDriveSubsystem swerveDrive, double targetAngleDegrees) {
    m_swerveDrive = swerveDrive;
    m_targetAngleDegrees = targetAngleDegrees;

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
    // Reset the PID controller
    m_pidController.reset();

    // Set the setpoint to the target angle
    m_pidController.setSetpoint(m_targetAngleDegrees);

    System.out.println("RotateToAngle: Starting rotation to " + m_targetAngleDegrees + " degrees");
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
      System.out.println("RotateToAngle: Command interrupted");
    } else {
      System.out.println("RotateToAngle: Reached target angle " + m_targetAngleDegrees + " degrees");
    }
  }

  @Override
  public boolean isFinished() {
    // Command is finished when we're within tolerance of the target angle
    return m_pidController.atSetpoint();
  }
}

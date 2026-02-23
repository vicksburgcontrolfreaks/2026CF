// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Command that rotates the robot to face a specific target position on the field.
 * The target position depends on the alliance color (blue or red).
 * Uses field coordinates and calculates the required heading angle.
 */
public class RotateToTargetCommand extends Command {
  private final SwerveDriveSubsystem m_swerveDrive;
  private final RobotContainer m_robotContainer;
  private final double m_toleranceDegrees = 1.0;
  private double m_targetAngle;

  /**
   * Creates a new RotateToTargetCommand.
   *
   * @param swerveDrive The swerve drive subsystem
   * @param robotContainer The robot container to access getSpeedLimit()
   */
  public RotateToTargetCommand(SwerveDriveSubsystem swerveDrive, RobotContainer robotContainer) {
    m_swerveDrive = swerveDrive;
    m_robotContainer = robotContainer;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    // Get current robot pose
    double robotX = m_swerveDrive.getPose().getX();
    double robotY = m_swerveDrive.getPose().getY();

    // Determine target coordinates based on alliance
    double targetX;
    double targetY;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      targetX = AutoConstants.kBlueTargetX;
      targetY = AutoConstants.kBlueTargetY;
    } else {
      targetX = AutoConstants.kRedTargetX;
      targetY = AutoConstants.kRedTargetY;
    }

    // Calculate angle to target
    double deltaX = targetX - robotX;
    double deltaY = targetY - robotY;
    m_targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
  }

  @Override
  public void execute() {
    // Calculate heading error
    double currentHeading = m_swerveDrive.getHeading();
    double headingError = Math.IEEEremainder(m_targetAngle - currentHeading, 360.0);

    // Calculate rotation speed with speed limit applied
    double rotationSpeed = SwerveConstants.kMaxAngularSpeedRadiansPerSecond
                         * m_robotContainer.getSpeedLimit();

    // Apply proportional control scaled by heading error
    // Use the sign of the error to determine rotation direction
    double rotationCommand = Math.signum(headingError) * rotationSpeed;

    // Drive with rotation only (no translation)
    m_swerveDrive.drive(0, 0, rotationCommand, false);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    m_swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    // Calculate heading error
    double currentHeading = m_swerveDrive.getHeading();
    double headingError = Math.abs(Math.IEEEremainder(m_targetAngle - currentHeading, 360.0));

    // Finish when within tolerance
    return headingError <= m_toleranceDegrees;
  }
}

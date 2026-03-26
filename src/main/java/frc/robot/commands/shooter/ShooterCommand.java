// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

/**
 * ShooterCommand - Shoot while maintaining alignment to speaker.
 *
 * When activated (mech controller right trigger):
 * - Takes over robot rotation control (driver translation still available)
 * - Maintains back of robot facing speaker
 * - Uses dynamic RPM based on distance
 * - Only feeds balls when aligned AND at target RPM
 *
 * Shooter is already spun up by teleopInit, so this command just manages
 * rotation control and feeding logic.
 */
public class ShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final DriveSubsystem m_drive;
  private final PIDController m_rotationController;
  private final RobotContainer m_container;
  private boolean m_feedingStarted = false;

  /**
   * Creates a new ShooterCommand.
   *
   * @param shooter ShooterSubsystem to control
   * @param vision PhotonVisionSubsystem (unused - kept for compatibility)
   * @param drive DriveSubsystem to control rotation
   * @param container RobotContainer for accessing PID tuning parameters
   */
  public ShooterCommand(ShooterSubsystem shooter, PhotonVisionSubsystem vision,
                        DriveSubsystem drive, RobotContainer container) {
    m_shooter = shooter;
    m_drive = drive;
    m_container = container;

    m_rotationController = new PIDController(
        AutoConstants.kRotateToTargetP,
        AutoConstants.kRotateToTargetI,
        AutoConstants.kRotateToTargetD
    );
    m_rotationController.enableContinuousInput(-180, 180);
    m_rotationController.setTolerance(AutoConstants.kRotateToTargetTolerance);

    addRequirements(shooter, drive);
  }

  @Override
  public void initialize() {
    m_feedingStarted = false;
    m_rotationController.reset();
    // Shooter is already active from teleopInit, just ensure it's running
    if (!m_shooter.isShooterActive()) {
      m_shooter.activateShooter();
    }
  }

  @Override
  public void execute() {
    // Update PID gains from NetworkTables (allows real-time tuning)
    double p = m_container.m_alignmentPEntry.get();
    double i = m_container.m_alignmentIEntry.get();
    double d = m_container.m_alignmentDEntry.get();
    double tolerance = m_container.m_alignmentToleranceEntry.get();

    m_rotationController.setPID(p, i, d);
    m_rotationController.setTolerance(tolerance);

    // Get current robot pose
    Pose2d currentPose = m_drive.getPose();

    // Determine target speaker based on alliance
    Translation2d targetPosition;
    if (DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == Alliance.Blue) {
      targetPosition = AutoConstants.blueTarget;
    } else {
      targetPosition = AutoConstants.redTarget;
    }

    // Calculate angle to target (back of robot facing speaker)
    double deltaX = targetPosition.getX() - currentPose.getX();
    double deltaY = targetPosition.getY() - currentPose.getY();
    double targetAngleDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));

    // Add 180 degrees to point the BACK of the robot at the target
    targetAngleDegrees = (targetAngleDegrees + 180) % 360;
    if (targetAngleDegrees > 180) {
      targetAngleDegrees -= 360;
    }

    // Get current heading
    double currentHeading = m_drive.getHeading();

    // Calculate rotation using PID
    double rotationSpeed = m_rotationController.calculate(currentHeading, targetAngleDegrees);

    // Clamp rotation speed
    rotationSpeed = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
                             Math.min(AutoConstants.kRotateToTargetMaxVelocity, rotationSpeed));

    // Apply only rotation control (no translation - driver keeps control)
    m_drive.drive(0, 0, rotationSpeed * DriveConstants.kMaxAngularSpeed, true);

    // Check if we should feed balls: aligned AND at target RPM
    boolean isAligned = m_rotationController.atSetpoint();
    boolean isAtRPM = m_shooter.isReadyToFeed();
    boolean shouldFeed = isAligned && isAtRPM;

    if (shouldFeed && !m_feedingStarted) {
      // Start feeding
      m_shooter.runIndexer(false);
      m_shooter.runFloor(false);
      m_feedingStarted = true;
    } else if (!shouldFeed && m_feedingStarted) {
      // Stop feeding (not aligned or not at RPM)
      m_shooter.StopIndexer();
      m_shooter.StopFloor();
      m_feedingStarted = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop feeding, but keep shooter spinning (stays active for quick re-shoot)
    m_shooter.StopFloor();
    m_shooter.StopIndexer();
    // Don't call m_drive.stop() - let the default command resume smoothly
    // This allows driver to immediately regain rotation control
  }

  @Override
  public boolean isFinished() {
    // Command runs while button is held
    return false;
  }
}

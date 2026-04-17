// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
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
 * - Automatically retracts hopper after 2 seconds to pull in straggler balls
 *
 * Shooter is already spun up by teleopInit, so this command just manages
 * rotation control and feeding logic.
 */
public class ShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final CollectorSubsystem m_collector;
  private final DriveSubsystem m_drive;
  private final CommandXboxController m_driverController;
  private final PIDController m_rotationController;
  private final RobotContainer m_container;
  private boolean m_feedingStarted = false;

  /**
   * Creates a new ShooterCommand.
   *
   * @param shooter ShooterSubsystem to control
   * @param collector CollectorSubsystem to control hopper
   * @param vision PhotonVisionSubsystem (unused - kept for compatibility)
   * @param drive DriveSubsystem to control rotation
   * @param driverController Driver controller for reading translation inputs
   * @param container RobotContainer for accessing PID tuning parameters
   */
  public ShooterCommand(ShooterSubsystem shooter, CollectorSubsystem collector,
                        PhotonVisionSubsystem vision, DriveSubsystem drive,
                        CommandXboxController driverController, RobotContainer container) {
    m_shooter = shooter;
    m_collector = collector;
    m_drive = drive;
    m_driverController = driverController;
    m_container = container;

    m_rotationController = new PIDController(
        AutoConstants.kRotateToTargetP,
        AutoConstants.kRotateToTargetI,
        AutoConstants.kRotateToTargetD
    );
    m_rotationController.enableContinuousInput(-180, 180);
    m_rotationController.setTolerance(AutoConstants.kRotateToTargetTolerance);

    // Only require shooter - let default drive command continue for translation
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_feedingStarted = false;
    m_rotationController.reset();
    // Shooter is already active from teleopInit, just ensure it's running
    // Use left/right shooters only (disable middle shooter for hardware issue)
    if (!m_shooter.isShooterActive()) {
      m_shooter.activateLeftRightShootersOnly();
    }
    // Remove RPM cap when trigger pulled - allow full distance-based RPM
    m_shooter.enableFullRPM();
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

    // If aligned, stop rotation control to prevent micro-corrections
    if (m_rotationController.atSetpoint()) {
      rotationSpeed = 0.0;
    } else {
      // Clamp rotation speed when not at setpoint
      rotationSpeed = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
                               Math.min(AutoConstants.kRotateToTargetMaxVelocity, rotationSpeed));
    }

    // Read driver's translation inputs (with speed multiplier logic)
    double speedMultiplier;
    if (m_driverController.rightBumper().getAsBoolean()) {
      speedMultiplier = OIConstants.kTurboSpeedLimit;
    } else if (m_driverController.leftBumper().getAsBoolean()) {
      speedMultiplier = OIConstants.kPrecisionSpeedLimit;
    } else {
      speedMultiplier = OIConstants.kNormalSpeedLimit;
    }

    double xSpeed = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond * speedMultiplier;
    double ySpeed = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond * speedMultiplier;

    // Alliance-relative driving: flip X and Y when on red alliance
    if (DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    // Apply rotation control from alignment + driver's translation control
    m_drive.drive(xSpeed, ySpeed, rotationSpeed * DriveConstants.kMaxAngularSpeed, true);

    // Check if we should feed balls: aligned AND at target RPM
    boolean isAligned = m_rotationController.atSetpoint();
    boolean isAtRPM = m_shooter.isReadyToFeed();
    boolean shouldFeed = isAligned && isAtRPM;

    if (shouldFeed && !m_feedingStarted) {
      // Start feeding - lower collector removed, only run indexer and floor
      m_shooter.runIndexer(false);
      m_shooter.runFloor(false);
      m_feedingStarted = true;
    }
    // Note: Once feeding starts, DON'T stop it due to minor alignment fluctuations
    // The feeding will continue until the command ends (trigger released)
    // This prevents jitter from PID micro-corrections
  }

  @Override
  public void end(boolean interrupted) {
    // Stop feeding, but keep shooter spinning (stays active for quick re-shoot)
    m_shooter.StopFloor();
    m_shooter.StopIndexer();
    m_collector.stopCollector();  // Stop collector
    // Re-enable RPM cap when trigger released - back to pre-spin mode
    m_shooter.enableRPMCap();
    // Don't call m_drive.stop() - let the default command resume smoothly
    // This allows driver to immediately regain rotation control
  }

  @Override
  public boolean isFinished() {
    // Command runs while button is held
    return false;
  }
}

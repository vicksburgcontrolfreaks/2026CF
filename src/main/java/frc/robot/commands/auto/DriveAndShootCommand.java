// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Autonomous command that drives away from the hub 1 meter, then shoots.
 *
 * Sequence:
 * 1. Drive +1m X (Red) or -1m X (Blue) from starting position
 * 2. Align to speaker
 * 3. Shoot when aligned and at target RPM
 */
public class DriveAndShootCommand extends Command {
  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final CollectorSubsystem m_collector;
  private final PIDController m_rotationController;

  private Pose2d m_startPose;
  private Pose2d m_targetDrivePose;
  private boolean m_driveComplete = false;
  private boolean m_shootingStarted = false;
  private double m_shootingStartTime = 0;
  private boolean m_hopperPopHigh = false;
  private double m_lastPopTime = 0;

  private static final double DRIVE_TOLERANCE = 0.1; // meters
  private static final double SHOOTING_DURATION = 3.0; // seconds

  public DriveAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
                              CollectorSubsystem collector) {
    m_drive = drive;
    m_shooter = shooter;
    m_collector = collector;

    m_rotationController = new PIDController(
      AutoConstants.kRotateToTargetP,
      AutoConstants.kRotateToTargetI,
      AutoConstants.kRotateToTargetD
    );
    m_rotationController.enableContinuousInput(-180, 180);
    m_rotationController.setTolerance(AutoConstants.kRotateToTargetTolerance);

    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    m_startPose = m_drive.getPose();
    m_driveComplete = false;
    m_shootingStarted = false;
    m_shootingStartTime = 0;
    m_rotationController.reset();

    // Calculate target position: drive 1m away from hub
    // Red alliance: +1m X, Blue alliance: -1m X
    boolean isRed = DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == Alliance.Red;
    double xOffset = isRed ? 1.0 : -1.0;

    m_targetDrivePose = new Pose2d(
      m_startPose.getX() + xOffset,
      m_startPose.getY(),
      m_startPose.getRotation()
    );

    // Shooter is already spinning from autonomousInit() at capped RPM (3500)
    // enableFullRPM() is called only when aiming begins in Phase 2
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();

    if (!m_driveComplete) {
      // Phase 1: Drive to target position
      double deltaX = m_targetDrivePose.getX() - currentPose.getX();
      double deltaY = m_targetDrivePose.getY() - currentPose.getY();
      double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

      if (distanceToTarget < DRIVE_TOLERANCE) {
        // Reached target position
        m_drive.drive(0, 0, 0, true);
        m_driveComplete = true;
      } else {
        // Drive toward target with simple proportional control (field-relative)
        double speed = Math.min(0.5, distanceToTarget * 2.0);
        double xSpeed = (deltaX / distanceToTarget) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeed = (deltaY / distanceToTarget) * speed * DriveConstants.kMaxSpeedMetersPerSecond;

        m_drive.drive(xSpeed, ySpeed, 0, true);
      }
    } else {
      // Phase 2: Aim and shoot (same logic as ShooterWithAutoAimCommand)
      boolean isRed = DriverStation.getAlliance().isPresent() &&
                      DriverStation.getAlliance().get() == Alliance.Red;
      Translation2d targetPosition = isRed ? AutoConstants.redTarget : AutoConstants.blueTarget;

      double deltaX = targetPosition.getX() - currentPose.getX();
      double deltaY = targetPosition.getY() - currentPose.getY();
      double targetAngleDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));
      targetAngleDegrees = (targetAngleDegrees + 180) % 360;
      if (targetAngleDegrees > 180) targetAngleDegrees -= 360;

      double rotationSpeed = m_rotationController.calculate(m_drive.getHeading(), targetAngleDegrees);
      rotationSpeed = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
                     Math.min( AutoConstants.kRotateToTargetMaxVelocity, rotationSpeed));

      m_drive.drive(0, 0, rotationSpeed * DriveConstants.kMaxAngularSpeed, true);

      // Remove RPM cap now that we're aiming — let distance-based RPM take over
      m_shooter.enableFullRPM();

      boolean isAligned = m_rotationController.atSetpoint();
      boolean isAtRPM = m_shooter.isReadyToFeed();

      if (isAligned && isAtRPM && !m_shootingStarted) {
        m_shooter.runIndexer(false);
        m_shooter.runFloor(false);
        m_collector.runCollector(false);
        m_collector.setHopperPosition(0.02);
        m_hopperPopHigh = false;
        m_lastPopTime = System.currentTimeMillis() / 1000.0;
        m_shootingStarted = true;
        m_shootingStartTime = System.currentTimeMillis() / 1000.0;
      }

      // Pop hopper while shooting
      if (m_shootingStarted) {
        double t = System.currentTimeMillis() / 1000.0;
        if (t - m_lastPopTime >= 0.25) {
          m_hopperPopHigh = !m_hopperPopHigh;
          m_collector.setHopperPosition(m_hopperPopHigh ? 0.19 : 0.02);
          m_lastPopTime = t;
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    m_shooter.StopFloor();
    m_shooter.StopIndexer();
    m_collector.stopCollector();
    m_shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    // Finish after shooting for the specified duration
    if (m_shootingStarted) {
      double currentTime = System.currentTimeMillis() / 1000.0;
      return (currentTime - m_shootingStartTime) >= SHOOTING_DURATION;
    }
    return false;
  }
}

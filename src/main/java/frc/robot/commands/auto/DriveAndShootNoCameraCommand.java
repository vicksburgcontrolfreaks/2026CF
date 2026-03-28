// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Camera-free fallback autonomous: drives 1m using odometry only (wheel encoders + gyro),
 * then shoots immediately at fixed RPM with no alignment or vision required.
 */
public class DriveAndShootNoCameraCommand extends Command {
  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final CollectorSubsystem m_collector;

  private Pose2d m_startPose;
  private boolean m_driveComplete = false;
  private boolean m_shootingStarted = false;
  private double m_shootingStartTime = 0;

  private static final double DRIVE_TOLERANCE    = 0.1;  // meters
  private static final double SHOOT_RPM          = 3500;
  private static final double SHOOTING_DURATION  = 3.0;  // seconds

  public DriveAndShootNoCameraCommand(DriveSubsystem drive, ShooterSubsystem shooter,
                                      CollectorSubsystem collector) {
    m_drive = drive;
    m_shooter = shooter;
    m_collector = collector;
    addRequirements(drive, shooter, collector);
  }

  @Override
  public void initialize() {
    m_startPose = m_drive.getPose();
    m_driveComplete = false;
    m_shootingStarted = false;
    m_shootingStartTime = 0;

    // Spin up to fixed RPM — no distance calculation, no vision
    m_shooter.activateShooterWithRPM(SHOOT_RPM);
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();

    if (!m_driveComplete) {
      boolean isRed = DriverStation.getAlliance().isPresent() &&
                      DriverStation.getAlliance().get() == Alliance.Red;
      double targetX = m_startPose.getX() + (isRed ? 1.0 : -1.0);

      double deltaX = targetX - currentPose.getX();
      double dist = Math.abs(deltaX);

      if (dist < DRIVE_TOLERANCE) {
        m_drive.drive(0, 0, 0, true);
        m_driveComplete = true;
      } else {
        double speed = Math.min(0.5, dist * 2.0);
        double xSpeed = Math.signum(deltaX) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
        m_drive.drive(xSpeed, 0, 0, true);
      }
    } else {
      // No aiming — shoot immediately at fixed RPM
      m_drive.drive(0, 0, 0, true);

      if (!m_shootingStarted && m_shooter.isReadyToFeed()) {
        m_shooter.runIndexer(false);
        m_shooter.runFloor(false);
        m_collector.runLowerCollectorRPM(1000);
        m_shootingStarted = true;
        m_shootingStartTime = System.currentTimeMillis() / 1000.0;
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
    if (m_shootingStarted) {
      return (System.currentTimeMillis() / 1000.0 - m_shootingStartTime) >= SHOOTING_DURATION;
    }
    return false;
  }
}

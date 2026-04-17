// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.ShootWithStartupCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Blue alliance RIGHT collect-and-shoot autonomous (lower field, y near 0.50).
 * Mirrors Red Right across the field.
 *
 * Sequence:
 *  1. Drive to (5.85, 0.59) @ 0°   — deploy hopper, start collector
 *  2. Drive to (7.76, 1.67) @ 90°  — collector running
 *  3. Drive to (7.76, 3.67) @ 90°  — collect; stop collector on arrival
 *  4. Drive to (5.85, 0.71) @ 180°
 *  5. Drive to (3.45, 0.59) @ 180°
 *  6. Drive to (3.21, 2.59) @ -60°
 *  7. Aim and shoot (4 sec)
 *  8. Drive to outpost (0.66, 0.62) @ 180° — near blue alliance wall
 *  9. Wait 2 seconds
 * 10. Drive back to (3.21, 2.59) @ -60°, aim and shoot again (4 sec)
 */
public class BlueRightCollectAndShootCommand extends Command {

  private enum Phase {
    DRIVE_TO_COLLECTOR_DEPLOY,
    DRIVE_TO_COLLECT_ALIGN,
    DRIVE_COLLECT,
    DRIVE_TO_MIDPOINT,
    DRIVE_TO_TRANSIT,
    DRIVE_TO_SHOOT,
    SHOOT,
    DRIVE_TO_OUTPOST,
    WAIT_AT_OUTPOST,
    DRIVE_TO_SHOOT_AGAIN,
    SHOOT_AGAIN,
    DONE
  }

  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final CollectorSubsystem m_collector;
  private final PIDController m_rotationController;
  private Command m_shootCommand;

  private static final Translation2d COLLECTOR_DEPLOY_POS = new Translation2d(5.85, 0.59);
  private static final Translation2d COLLECT_ALIGN_POS    = new Translation2d(7.76, 1.67);
  private static final Translation2d COLLECT_POS          = new Translation2d(7.76, 3.67);
  private static final Translation2d MIDPOINT_POS         = new Translation2d(5.85, 0.71);
  private static final Translation2d TRANSIT_POS          = new Translation2d(3.45, 0.59);
  private static final Translation2d SHOOT_POS            = new Translation2d(3.21, 2.59);
  private static final Translation2d OUTPOST_POS          = new Translation2d(0.66, 0.62);
  private static final double DRIVE_TOLERANCE             = 0.15;
  private static final double SHOOT_DURATION              = 4.0;
  private static final double OUTPOST_WAIT_DURATION       = 2.0;

  private Phase m_phase;
  private double m_waitStartTime;

  public BlueRightCollectAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
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

    addRequirements(drive, shooter, collector);
  }

  @Override
  public void initialize() {
    m_phase = Phase.DRIVE_TO_COLLECTOR_DEPLOY;
    m_shootCommand = null;
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    double t = now();
    Pose2d pose = m_drive.getPose();

    switch (m_phase) {

      case DRIVE_TO_COLLECTOR_DEPLOY:
        driveToWaypoint(pose, COLLECTOR_DEPLOY_POS, 0.0, Phase.DRIVE_TO_COLLECT_ALIGN);
        if (m_phase == Phase.DRIVE_TO_COLLECT_ALIGN) {
          m_collector.extendHopper();
          m_collector.runCollector(false);
        }
        break;

      case DRIVE_TO_COLLECT_ALIGN:
        driveToWaypoint(pose, COLLECT_ALIGN_POS, 90.0, Phase.DRIVE_COLLECT);
        break;

      case DRIVE_COLLECT:
        driveToWaypoint(pose, COLLECT_POS, 90.0, Phase.DRIVE_TO_MIDPOINT);
        if (m_phase == Phase.DRIVE_TO_MIDPOINT) {
          m_collector.stopCollector();
        }
        break;

      case DRIVE_TO_MIDPOINT:
        driveToWaypoint(pose, MIDPOINT_POS, 180.0, Phase.DRIVE_TO_TRANSIT);
        break;

      case DRIVE_TO_TRANSIT:
        driveToWaypoint(pose, TRANSIT_POS, 180.0, Phase.DRIVE_TO_SHOOT);
        break;

      case DRIVE_TO_SHOOT:
        driveToWaypoint(pose, SHOOT_POS, -60.0, Phase.SHOOT);
        break;

      case SHOOT:
        if (m_shootCommand == null) {
          m_shootCommand = new ShootWithStartupCommand(m_shooter, m_drive, m_collector)
            .withTimeout(SHOOT_DURATION);
          m_shootCommand.schedule();
        }
        if (m_shootCommand.isFinished()) {
          m_shootCommand = null;
          m_phase = Phase.DRIVE_TO_OUTPOST;
        }
        break;

      case DRIVE_TO_OUTPOST:
        driveToWaypoint(pose, OUTPOST_POS, 180.0, Phase.WAIT_AT_OUTPOST);
        if (m_phase == Phase.WAIT_AT_OUTPOST) {
          m_waitStartTime = now();
        }
        break;

      case WAIT_AT_OUTPOST:
        m_drive.drive(0, 0, 0, true);
        if ((t - m_waitStartTime) >= OUTPOST_WAIT_DURATION) {
          m_phase = Phase.DRIVE_TO_SHOOT_AGAIN;
        }
        break;

      case DRIVE_TO_SHOOT_AGAIN:
        driveToWaypoint(pose, SHOOT_POS, -60.0, Phase.SHOOT_AGAIN);
        break;

      case SHOOT_AGAIN:
        if (m_shootCommand == null) {
          m_shootCommand = new ShootWithStartupCommand(m_shooter, m_drive, m_collector)
            .withTimeout(SHOOT_DURATION);
          m_shootCommand.schedule();
        }
        if (m_shootCommand.isFinished()) {
          m_shootCommand = null;
          m_phase = Phase.DONE;
        }
        break;

      case DONE:
        m_drive.drive(0, 0, 0, true);
        break;
    }
  }

  private void driveToWaypoint(Pose2d pose, Translation2d waypoint, double targetHeading,
                                Phase nextPhase) {
    double dx = waypoint.getX() - pose.getX();
    double dy = waypoint.getY() - pose.getY();
    double dist = Math.sqrt(dx * dx + dy * dy);

    double rot = m_rotationController.calculate(m_drive.getHeading(), targetHeading);
    rot = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
          Math.min( AutoConstants.kRotateToTargetMaxVelocity, rot));

    if (dist < DRIVE_TOLERANCE) {
      m_drive.drive(0, 0, rot * DriveConstants.kMaxAngularSpeed, true);
      if (m_rotationController.atSetpoint()) {
        m_drive.drive(0, 0, 0, true);
        m_rotationController.reset();
        m_phase = nextPhase;
      }
    } else {
      double speed = Math.min(0.5, dist * 2.0);
      double xSpeed = (dx / dist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeed = (dy / dist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      m_drive.drive(xSpeed, ySpeed, rot * DriveConstants.kMaxAngularSpeed, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    m_shooter.StopFloor();
    m_shooter.StopIndexer();
    m_shooter.stopShooter();
    m_collector.stopCollector();
    if (m_shootCommand != null) {
      m_shootCommand.cancel();
      m_shootCommand = null;
    }
  }

  @Override
  public boolean isFinished() {
    return m_phase == Phase.DONE;
  }

  private static double now() {
    return System.currentTimeMillis() / 1000.0;
  }
}

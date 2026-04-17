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
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

/**
 * Red alliance RIGHT collect-and-shoot autonomous — OVER HUMP variant.
 * Returns over the hump both times instead of going under the trench.
 *
 * Sequence:
 * 1.  Drive to (10.69, 7.62) @ 180° — deploy hopper, start collector
 * 2.  Drive to (9.4,   6.5)  @ -135°
 * 3.  Drive to (8.8,   5.5)  @ -90°  — collecting
 * 4.  Drive to (9.5,   4.5)  @ -45°
 * 5.  Drive to (10.3,  5.7)  @ 45°   — pre-hump
 * 6.  Drive over hump to (13.5, 5.6) @ 45°
 *     SETTLE — hold position until multi-tag pose (≥2 tags) or 1.5s timeout
 * 7.  Aim and shoot (2.5 sec)
 * 8.  Drive back over hump to (10.3, 5.7) @ 180°
 *     SETTLE — hold position until multi-tag pose (≥2 tags) or 1.5s timeout
 * 9.  Drive back to (9.4, 6.5) @ -90° — redeploy hopper, restart collector
 * 10. Drive to (8.8,  5.5)  @ -90°
 * 11. Drive to (9.5,  4.5)  @ -45°
 * 12. Drive to (10.3, 5.7)  @ 45°
 * 13. Drive over hump to (13.5, 5.6) @ 45°
 *     SETTLE — hold position until multi-tag pose (≥2 tags) or 1.5s timeout
 * 14. Aim and shoot again (2.5 sec)
 *
 * Odometry note: the hump crossing causes wheel slip. Vision settle phases
 * stop the robot and wait for ≥2 AprilTags to correct the pose before
 * proceeding. A 1.5s timeout prevents the robot from getting stuck if tags
 * are not visible.
 */
public class RedRightOverHumpCommand extends Command {

  private enum Phase {
    DRIVE_TO_COLLECTOR_DEPLOY,
    DRIVE_TO_COLLECT_ALIGN,
    DRIVE_COLLECT,
    DRIVE_SWEEP_MID,
    DRIVE_PRE_HUMP,
    DRIVE_OVER_HUMP,
    VISION_SETTLE,
    SHOOT,
    DRIVE_BACK_OVER_HUMP,
    VISION_SETTLE_RETURN,
    DRIVE_BACK_TO_COLLECT,
    DRIVE_INTERMEDIATE_2,
    DRIVE_COLLECT_2,
    DRIVE_SWEEP_MID_2,
    DRIVE_PRE_HUMP_2,
    DRIVE_OVER_HUMP_2,
    VISION_SETTLE_2,
    SHOOT_AGAIN,
    DONE
  }

  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final CollectorSubsystem m_collector;
  private final PhotonVisionSubsystem m_vision;
  private final PIDController m_rotationController;
  private Command m_shootCommand;

  private static final Translation2d COLLECTOR_DEPLOY_POS = new Translation2d(10.69, 7.5);
  private static final Translation2d COLLECT_ALIGN_POS    = new Translation2d(9.4,   6.5);
  private static final Translation2d COLLECT_POS          = new Translation2d(8.8,   5.5);
  private static final Translation2d SWEEP_MID_POS        = new Translation2d(9.5,   4.5);
  private static final Translation2d PRE_HUMP_POS         = new Translation2d(10.3,  5.7);
  private static final Translation2d SHOOT_POS            = new Translation2d(14.2,  5.6);

  // Second cycle waypoints (different path after first shoot)
  private static final Translation2d COLLECT_ALIGN_POS_2  = new Translation2d(8.7,   5.15);
  private static final Translation2d INTERMEDIATE_POS_2   = new Translation2d(8.8,   3.9);
  private static final Translation2d COLLECT_POS_2        = new Translation2d(10.0,  3.9);
  private static final Translation2d SWEEP_MID_POS_2      = new Translation2d(10.6,  4.7);
  private static final Translation2d PRE_HUMP_POS_2       = new Translation2d(10.6,  5.6);
  private static final double DRIVE_TOLERANCE             = 0.25;
  private static final double SMOOTH_TOLERANCE            = 0.5;    // Larger tolerance for smooth transitions
  private static final double LOOKAHEAD_DISTANCE          = 0.75;   // Start turning toward next waypoint early
  private static final double HUMP_SPEED                  = 0.5;
  private static final double COLLECTION_MAX_SPEED        = 0.85;   // Higher speed for collection sequence
  private static final double SHOOT_DURATION              = 3.5;
  private static final int    VISION_SETTLE_MIN_TAGS      = 2;
  private static final double VISION_SETTLE_TIMEOUT       = 1.5;

  private Phase m_phase;
  private double m_settleStartTime;

  public RedRightOverHumpCommand(DriveSubsystem drive, ShooterSubsystem shooter,
                                 CollectorSubsystem collector, PhotonVisionSubsystem vision) {
    m_drive = drive;
    m_shooter = shooter;
    m_collector = collector;
    m_vision = vision;

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
    m_settleStartTime = 0;
    m_rotationController.reset();

    // Start collector motor at the beginning and keep it running throughout autonomous
    m_collector.runCollector(false);
  }

  @Override
  public void execute() {
    double t = now();
    Pose2d pose = m_drive.getPose();

    switch (m_phase) {

      case DRIVE_TO_COLLECTOR_DEPLOY:
        m_collector.extendHopper();  // Deploy hopper while driving
        driveToWaypointSmooth(pose, COLLECTOR_DEPLOY_POS, COLLECT_ALIGN_POS, 180.0, Phase.DRIVE_TO_COLLECT_ALIGN);
        break;

      case DRIVE_TO_COLLECT_ALIGN:
        driveToWaypointSmooth(pose, COLLECT_ALIGN_POS, COLLECT_POS, -135.0, Phase.DRIVE_COLLECT);
        break;

      case DRIVE_COLLECT:
        driveToWaypointSmooth(pose, COLLECT_POS, SWEEP_MID_POS, -90.0, Phase.DRIVE_SWEEP_MID);
        break;

      case DRIVE_SWEEP_MID:
        driveToWaypoint(pose, SWEEP_MID_POS, -45.0, Phase.DRIVE_PRE_HUMP);
        break;

      case DRIVE_PRE_HUMP:
        driveToWaypoint(pose, PRE_HUMP_POS, 45.0, Phase.DRIVE_OVER_HUMP);
        break;

      case DRIVE_OVER_HUMP:
        driveOverHump(pose, SHOOT_POS, 45.0, Phase.VISION_SETTLE);
        if (m_phase == Phase.VISION_SETTLE) {
          m_settleStartTime = now();
        }
        break;

      case VISION_SETTLE:
        m_drive.drive(0, 0, 0, true);
        if (m_vision.getTotalTagCount() >= VISION_SETTLE_MIN_TAGS ||
            (t - m_settleStartTime) >= VISION_SETTLE_TIMEOUT) {
          m_phase = Phase.SHOOT;
        }
        break;

      case SHOOT:
        if (m_shootCommand == null) {
          m_shootCommand = new ShootWithStartupCommand(m_shooter, m_drive, m_collector)
            .withTimeout(SHOOT_DURATION);
          m_shootCommand.schedule();
        }
        if (m_shootCommand.isFinished()) {
          m_shootCommand = null;
          m_phase = Phase.DRIVE_BACK_OVER_HUMP;
        }
        break;

      case DRIVE_BACK_OVER_HUMP:
        driveOverHump(pose, PRE_HUMP_POS, 180.0, Phase.VISION_SETTLE_RETURN);
        if (m_phase == Phase.VISION_SETTLE_RETURN) {
          m_settleStartTime = now();
        }
        break;

      case VISION_SETTLE_RETURN:
        m_drive.drive(0, 0, 0, true);
        if (m_vision.getTotalTagCount() >= VISION_SETTLE_MIN_TAGS ||
            (t - m_settleStartTime) >= VISION_SETTLE_TIMEOUT) {
          m_phase = Phase.DRIVE_BACK_TO_COLLECT;
        }
        break;

      case DRIVE_BACK_TO_COLLECT:
        m_collector.extendHopper();  // Deploy hopper while driving
        driveToWaypointSmooth(pose, COLLECT_ALIGN_POS_2, INTERMEDIATE_POS_2, -120.0, Phase.DRIVE_INTERMEDIATE_2);
        break;

      case DRIVE_INTERMEDIATE_2:
        driveToWaypointSmooth(pose, INTERMEDIATE_POS_2, COLLECT_POS_2, -45.0, Phase.DRIVE_COLLECT_2);
        break;

      case DRIVE_COLLECT_2:
        driveToWaypointSmooth(pose, COLLECT_POS_2, SWEEP_MID_POS_2, 40.0, Phase.DRIVE_SWEEP_MID_2);
        break;

      case DRIVE_SWEEP_MID_2:
        driveToWaypoint(pose, SWEEP_MID_POS_2, 90.0, Phase.DRIVE_PRE_HUMP_2);
        break;

      case DRIVE_PRE_HUMP_2:
        driveToWaypoint(pose, PRE_HUMP_POS_2, 45.0, Phase.DRIVE_OVER_HUMP_2);
        break;

      case DRIVE_OVER_HUMP_2:
        driveOverHump(pose, SHOOT_POS, 45.0, Phase.VISION_SETTLE_2);
        if (m_phase == Phase.VISION_SETTLE_2) {
          m_settleStartTime = now();
        }
        break;

      case VISION_SETTLE_2:
        m_drive.drive(0, 0, 0, true);
        if (m_vision.getTotalTagCount() >= VISION_SETTLE_MIN_TAGS ||
            (t - m_settleStartTime) >= VISION_SETTLE_TIMEOUT) {
          m_phase = Phase.SHOOT_AGAIN;
        }
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
      m_rotationController.reset();
      m_phase = nextPhase;
    } else {
      double speed = Math.min(0.5, dist * 2.0);
      double xSpeed = (dx / dist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeed = (dy / dist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      m_drive.drive(xSpeed, ySpeed, rot * DriveConstants.kMaxAngularSpeed, true);
    }
  }

  /**
   * Smooth waypoint following with lookahead for faster, flowing motion.
   * Uses larger tolerance and higher speeds for smooth transitions between waypoints.
   * Implements lookahead to start turning toward next waypoint before reaching current one.
   */
  private void driveToWaypointSmooth(Pose2d pose, Translation2d currentWaypoint,
                                      Translation2d nextWaypoint, double targetHeading,
                                      Phase nextPhase) {
    double dx = currentWaypoint.getX() - pose.getX();
    double dy = currentWaypoint.getY() - pose.getY();
    double dist = Math.sqrt(dx * dx + dy * dy);

    // Use lookahead: blend current and next waypoint when close to current
    Translation2d targetPoint = currentWaypoint;
    if (nextWaypoint != null && dist < LOOKAHEAD_DISTANCE) {
      // Blend between current and next waypoint for smooth cornering
      double blendFactor = 1.0 - (dist / LOOKAHEAD_DISTANCE);  // 0 when far, 1 when at waypoint
      targetPoint = new Translation2d(
        currentWaypoint.getX() * (1 - blendFactor) + nextWaypoint.getX() * blendFactor,
        currentWaypoint.getY() * (1 - blendFactor) + nextWaypoint.getY() * blendFactor
      );
    }

    // Recalculate direction to target point (may be blended)
    dx = targetPoint.getX() - pose.getX();
    dy = targetPoint.getY() - pose.getY();
    double targetDist = Math.sqrt(dx * dx + dy * dy);

    // Rotation control
    double rot = m_rotationController.calculate(m_drive.getHeading(), targetHeading);
    rot = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
          Math.min( AutoConstants.kRotateToTargetMaxVelocity, rot));

    // Transition to next phase when close enough (larger tolerance for smooth flow)
    if (dist < SMOOTH_TOLERANCE) {
      m_rotationController.reset();
      m_phase = nextPhase;
    } else {
      // Higher speed limit and smoother acceleration
      double speed = Math.min(COLLECTION_MAX_SPEED, dist * 1.5);
      double xSpeed = (dx / targetDist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeed = (dy / targetDist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      m_drive.drive(xSpeed, ySpeed, rot * DriveConstants.kMaxAngularSpeed, true);
    }
  }

  /** Same as driveToWaypoint but caps speed to HUMP_SPEED to reduce wheel slip over the bump. */
  private void driveOverHump(Pose2d pose, Translation2d waypoint, double targetHeading,
                              Phase nextPhase) {
    double dx = waypoint.getX() - pose.getX();
    double dy = waypoint.getY() - pose.getY();
    double dist = Math.sqrt(dx * dx + dy * dy);

    double rot = m_rotationController.calculate(m_drive.getHeading(), targetHeading);
    rot = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
          Math.min( AutoConstants.kRotateToTargetMaxVelocity, rot));

    if (dist < DRIVE_TOLERANCE) {
      m_rotationController.reset();
      m_phase = nextPhase;
    } else {
      double speed = Math.min(HUMP_SPEED, dist * 2.0);
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

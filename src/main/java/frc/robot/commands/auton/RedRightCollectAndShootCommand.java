// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

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
 * Red alliance RIGHT collect-and-shoot autonomous (upper field, y near 7.62).
 *
 * Sequence:
 * 1.  Drive to (10.69, 7.62) @ 180° — deploy hopper, start collector
 * 2.  Drive to (8.78,  6.54) @ -90° — collector running
 * 3.  Drive to (8.78,  5.78) @ -90° — collecting balls
 * 4a. Sweep   to (9.68,  5.78) @ 180° — CW sweep mid, collector still running
 * 4b. Sweep   to (10.58, 5.78) @ 90°  — CW sweep end, collector still running
 * 4c. Drive to (10.78, 7.50)  @ 0°   — crash avoidance
 * 5.  Drive to (12.88, 7.62) @ 0°  — collector still running
 * 6.  Drive to (13.33, 5.62) @ 60°
 * 7.  Aim and shoot (4 sec)
 * 8.  Drive back to (8.78, 6.54) @ -90° — redeploy hopper, restart collector
 * 9.  Drive to (8.78,  5.78) @ -90° — collecting
 * 10a.Sweep   to (9.68,  5.78) @ 180° — CW sweep mid
 * 10b.Sweep   to (10.58, 5.78) @ 90°  — CW sweep end
 * 10c.Drive to (10.78, 7.50)  @ 0°   — crash avoidance
 * 11. Drive to (12.88, 7.62) @ 0°
 * 12. Drive to (13.33, 5.62) @ 60°
 * 13. Aim and shoot again (4 sec)
 */
public class RedRightCollectAndShootCommand extends Command {

  private enum Phase {
    DRIVE_TO_COLLECTOR_DEPLOY,
    DRIVE_TO_COLLECT_ALIGN,
    DRIVE_COLLECT,
    DRIVE_SWEEP_MID,
    DRIVE_SWEEP,
    DRIVE_AVOID,
    DRIVE_TO_TRANSIT,
    DRIVE_TO_SHOOT,
    SHOOT,
    DRIVE_TO_TRENCH_ENTRY,
    DRIVE_THROUGH_TRENCH,
    DRIVE_BACK_TO_COLLECT,
    DRIVE_COLLECT_2,
    DRIVE_SWEEP_MID_2,
    DRIVE_SWEEP_2,
    DRIVE_AVOID_2,
    DRIVE_TO_TRANSIT_2,
    DRIVE_TO_SHOOT_AGAIN,
    SHOOT_AGAIN,
    DONE
  }

  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final CollectorSubsystem m_collector;
  private final PIDController m_rotationController;

  private static final Translation2d COLLECTOR_DEPLOY_POS = new Translation2d(10.69, 7.62);
  private static final Translation2d COLLECT_ALIGN_POS    = new Translation2d(8.78,  6.54);
  private static final Translation2d COLLECT_POS          = new Translation2d(8.78,  5.78);
  private static final Translation2d SWEEP_MID_POS        = new Translation2d(9.68,  5.00);
  private static final Translation2d SWEEP_POS            = new Translation2d(10.58, 5.78);
  private static final Translation2d AVOID_POS            = new Translation2d(10.78, 7.50);
  private static final Translation2d TRENCH_ENTRY_POS     = new Translation2d(13.33, 7.62);
  private static final Translation2d TRENCH_EXIT_POS      = new Translation2d(10.62, 7.62);
  private static final Translation2d TRANSIT_POS          = new Translation2d(12.88, 7.62);
  private static final Translation2d SHOOT_POS            = new Translation2d(13.33, 6.68);
  private static final double DRIVE_TOLERANCE             = 0.25;
  private static final double SHOOT_DURATION              = 4.0;

  private Phase m_phase;
  private boolean m_shootingStarted;
  private double m_shootingStartTime;
  private boolean m_hopperPopHigh;
  private double m_lastPopTime;

  public RedRightCollectAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
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
    m_shootingStarted = false;
    m_shootingStartTime = 0;
    m_hopperPopHigh = false;
    m_lastPopTime = 0;
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    double t = now();
    Pose2d pose = m_drive.getPose();

    switch (m_phase) {

      case DRIVE_TO_COLLECTOR_DEPLOY:
        driveToWaypoint(pose, COLLECTOR_DEPLOY_POS, 180.0, Phase.DRIVE_TO_COLLECT_ALIGN);
        if (m_phase == Phase.DRIVE_TO_COLLECT_ALIGN) {
          m_collector.extendHopper();
          m_collector.runCollector(false);
        }
        break;

      case DRIVE_TO_COLLECT_ALIGN:
        driveToWaypoint(pose, COLLECT_ALIGN_POS, -90.0, Phase.DRIVE_COLLECT);
        break;

      case DRIVE_COLLECT:
        driveToWaypoint(pose, COLLECT_POS, -90.0, Phase.DRIVE_SWEEP_MID);
        break;

      case DRIVE_SWEEP_MID:
        driveToWaypoint(pose, SWEEP_MID_POS, 0.0, Phase.DRIVE_SWEEP);
        break;

      case DRIVE_SWEEP:
        driveToWaypoint(pose, SWEEP_POS, 90.0, Phase.DRIVE_AVOID);
        break;

      case DRIVE_AVOID:
        driveToWaypoint(pose, AVOID_POS, 0.0, Phase.DRIVE_TO_TRANSIT);
        break;

      case DRIVE_TO_TRANSIT:
        driveToWaypoint(pose, TRANSIT_POS, 0.0, Phase.DRIVE_TO_SHOOT);
        break;

      case DRIVE_TO_SHOOT:
        driveToWaypoint(pose, SHOOT_POS, 60.0, Phase.SHOOT);
        break;

      case SHOOT:
        aimAndShoot(pose);
        if (m_shootingStarted && (t - m_shootingStartTime) >= SHOOT_DURATION) {
          m_shooter.StopFloor();
          m_shooter.StopIndexer();
          m_shooter.enableRPMCap();
          m_collector.stopCollector();
          m_shootingStarted = false;
          m_hopperPopHigh = false;
          m_lastPopTime = 0;
          m_rotationController.reset();
          m_phase = Phase.DRIVE_TO_TRENCH_ENTRY;
        }
        break;

      case DRIVE_TO_TRENCH_ENTRY:
        driveToWaypoint(pose, TRENCH_ENTRY_POS, 180.0, Phase.DRIVE_THROUGH_TRENCH);
        break;

      case DRIVE_THROUGH_TRENCH:
        driveToWaypoint(pose, TRENCH_EXIT_POS, 180.0, Phase.DRIVE_BACK_TO_COLLECT);
        break;

      case DRIVE_BACK_TO_COLLECT:
        driveToWaypoint(pose, COLLECT_ALIGN_POS, -90.0, Phase.DRIVE_COLLECT_2);
        if (m_phase == Phase.DRIVE_COLLECT_2) {
          m_collector.extendHopper();
          m_collector.runCollector(false);
        }
        break;

      case DRIVE_COLLECT_2:
        driveToWaypoint(pose, COLLECT_POS, -90.0, Phase.DRIVE_SWEEP_MID_2);
        break;

      case DRIVE_SWEEP_MID_2:
        driveToWaypoint(pose, SWEEP_MID_POS, 180.0, Phase.DRIVE_SWEEP_2);
        break;

      case DRIVE_SWEEP_2:
        driveToWaypoint(pose, SWEEP_POS, 90.0, Phase.DRIVE_AVOID_2);
        break;

      case DRIVE_AVOID_2:
        driveToWaypoint(pose, AVOID_POS, 0.0, Phase.DRIVE_TO_TRANSIT_2);
        break;

      case DRIVE_TO_TRANSIT_2:
        driveToWaypoint(pose, TRANSIT_POS, 0.0, Phase.DRIVE_TO_SHOOT_AGAIN);
        break;

      case DRIVE_TO_SHOOT_AGAIN:
        driveToWaypoint(pose, SHOOT_POS, 60.0, Phase.SHOOT_AGAIN);
        break;

      case SHOOT_AGAIN:
        aimAndShoot(pose);
        if (m_shootingStarted && (t - m_shootingStartTime) >= SHOOT_DURATION) {
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

  private void aimAndShoot(Pose2d pose) {
    boolean isRed = DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == Alliance.Red;
    Translation2d target = isRed ? AutoConstants.redTarget : AutoConstants.blueTarget;

    double dx = target.getX() - pose.getX();
    double dy = target.getY() - pose.getY();
    double targetAngle = Math.toDegrees(Math.atan2(dy, dx)) + 180;
    if (targetAngle > 180) targetAngle -= 360;

    double rot = m_rotationController.calculate(m_drive.getHeading(), targetAngle);
    rot = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
          Math.min( AutoConstants.kRotateToTargetMaxVelocity, rot));

    m_shooter.enableFullRPM();
    m_drive.drive(0, 0, rot * DriveConstants.kMaxAngularSpeed, true);

    if (!m_shootingStarted && m_rotationController.atSetpoint()) {
      m_shooter.runIndexer(false);
      m_shooter.runFloor(false);
      m_collector.runCollector(false);
      m_collector.setHopperPosition(0.02);
      m_hopperPopHigh = false;
      m_lastPopTime = now();
      m_shootingStarted = true;
      m_shootingStartTime = now();
    }

    if (m_shootingStarted) {
      double t = now();
      if (t - m_lastPopTime >= 0.25) {
        m_hopperPopHigh = !m_hopperPopHigh;
        m_collector.setHopperPosition(m_hopperPopHigh ? 0.19 : 0.02);
        m_lastPopTime = t;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    m_shooter.StopFloor();
    m_shooter.StopIndexer();
    m_shooter.stopShooter();
    m_collector.stopCollector();
  }

  @Override
  public boolean isFinished() {
    return m_phase == Phase.DONE;
  }

  private static double now() {
    return System.currentTimeMillis() / 1000.0;
  }
}

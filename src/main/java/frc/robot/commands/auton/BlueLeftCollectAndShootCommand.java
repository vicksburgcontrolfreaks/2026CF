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
 * Blue alliance LEFT collect-and-shoot autonomous (upper field, y near 7.62).
 * Mirrors Red Left across the field.
 *
 * Sequence:
 *  1. Drive to (5.84, 7.75) @ 0°    — through trench; deploy hopper, start collector
 *  2. Drive to (7.77, 6.16) @ -90°  — collector running
 *  3. Drive to (7.77, 4.68) @ -90°  — collect; stop collector on arrival
 *  4. Drive to (5.84, 7.75) @ 180°  — back through trench
 *  5. Drive to (3.60, 7.62) @ 180°  — clear trench
 *  6. Drive to (3.31, 5.84) @ 120°  — shoot position, facing SE toward blue speaker
 *  7. Aim and shoot (4 sec)
 *  8. Drive to (3.60, 7.62) @ 0°    — back to trench entrance
 *  9. Drive to (5.84, 7.75) @ 0°    — through trench; deploy hopper, start collector
 * 10. Drive to (7.77, 6.16) @ -90°
 * 11. Drive to (7.77, 4.68) @ -90°  — collect; stop collector on arrival
 * 12. Drive to (5.84, 7.75) @ 180°
 * 13. Drive to (3.60, 7.62) @ 180°
 * 14. Drive to (3.31, 5.84) @ 120°
 * 15. Aim and shoot again (4 sec)
 */
public class BlueLeftCollectAndShootCommand extends Command {

  private enum Phase {
    DRIVE_THROUGH_TRENCH_1,
    DRIVE_TO_COLLECT_ALIGN_1,
    DRIVE_COLLECT_1,
    DRIVE_BACK_THROUGH_TRENCH_1,
    DRIVE_CLEAR_TRENCH_1,
    DRIVE_TO_SHOOT_1,
    SHOOT_1,
    DRIVE_TO_TRENCH_ENTRANCE,
    DRIVE_THROUGH_TRENCH_2,
    DRIVE_TO_COLLECT_ALIGN_2,
    DRIVE_COLLECT_2,
    DRIVE_BACK_THROUGH_TRENCH_2,
    DRIVE_CLEAR_TRENCH_2,
    DRIVE_TO_SHOOT_2,
    SHOOT_2,
    DONE
  }

  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final CollectorSubsystem m_collector;
  private final PIDController m_rotationController;
  private Command m_shootCommand;

  private static final Translation2d TRENCH_FAR_POS    = new Translation2d(5.84, 7.75);
  private static final Translation2d COLLECT_ALIGN_POS = new Translation2d(7.77, 6.16);
  private static final Translation2d COLLECT_POS       = new Translation2d(7.77, 4.68);
  private static final Translation2d TRENCH_CLEAR_POS  = new Translation2d(3.60, 7.62);
  private static final Translation2d SHOOT_POS         = new Translation2d(3.31, 5.84);
  private static final double DRIVE_TOLERANCE          = 0.15;
  private static final double SHOOT_DURATION           = 4.0;

  private Phase m_phase;

  public BlueLeftCollectAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
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
    m_phase = Phase.DRIVE_THROUGH_TRENCH_1;
    m_shootCommand = null;
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    double t = now();
    Pose2d pose = m_drive.getPose();

    switch (m_phase) {

      case DRIVE_THROUGH_TRENCH_1:
        driveToWaypoint(pose, TRENCH_FAR_POS, 0.0, Phase.DRIVE_TO_COLLECT_ALIGN_1);
        if (m_phase == Phase.DRIVE_TO_COLLECT_ALIGN_1) {
          m_collector.extendHopper();
          m_collector.runCollector(false);
        }
        break;

      case DRIVE_TO_COLLECT_ALIGN_1:
        driveToWaypoint(pose, COLLECT_ALIGN_POS, -90.0, Phase.DRIVE_COLLECT_1);
        break;

      case DRIVE_COLLECT_1:
        driveToWaypoint(pose, COLLECT_POS, -90.0, Phase.DRIVE_BACK_THROUGH_TRENCH_1);
        if (m_phase == Phase.DRIVE_BACK_THROUGH_TRENCH_1) {
          m_collector.stopCollector();
        }
        break;

      case DRIVE_BACK_THROUGH_TRENCH_1:
        driveToWaypoint(pose, TRENCH_FAR_POS, 180.0, Phase.DRIVE_CLEAR_TRENCH_1);
        break;

      case DRIVE_CLEAR_TRENCH_1:
        driveToWaypoint(pose, TRENCH_CLEAR_POS, 180.0, Phase.DRIVE_TO_SHOOT_1);
        break;

      case DRIVE_TO_SHOOT_1:
        driveToWaypoint(pose, SHOOT_POS, 120.0, Phase.SHOOT_1);
        break;

      case SHOOT_1:
        if (m_shootCommand == null) {
          m_shootCommand = new ShootWithStartupCommand(m_shooter, m_drive, m_collector)
            .withTimeout(SHOOT_DURATION);
          m_shootCommand.schedule();
        }
        if (m_shootCommand.isFinished()) {
          m_shootCommand = null;
          m_phase = Phase.DRIVE_TO_TRENCH_ENTRANCE;
        }
        break;

      case DRIVE_TO_TRENCH_ENTRANCE:
        driveToWaypoint(pose, TRENCH_CLEAR_POS, 0.0, Phase.DRIVE_THROUGH_TRENCH_2);
        break;

      case DRIVE_THROUGH_TRENCH_2:
        driveToWaypoint(pose, TRENCH_FAR_POS, 0.0, Phase.DRIVE_TO_COLLECT_ALIGN_2);
        if (m_phase == Phase.DRIVE_TO_COLLECT_ALIGN_2) {
          m_collector.extendHopper();
          m_collector.runCollector(false);
        }
        break;

      case DRIVE_TO_COLLECT_ALIGN_2:
        driveToWaypoint(pose, COLLECT_ALIGN_POS, -90.0, Phase.DRIVE_COLLECT_2);
        break;

      case DRIVE_COLLECT_2:
        driveToWaypoint(pose, COLLECT_POS, -90.0, Phase.DRIVE_BACK_THROUGH_TRENCH_2);
        if (m_phase == Phase.DRIVE_BACK_THROUGH_TRENCH_2) {
          m_collector.stopCollector();
        }
        break;

      case DRIVE_BACK_THROUGH_TRENCH_2:
        driveToWaypoint(pose, TRENCH_FAR_POS, 180.0, Phase.DRIVE_CLEAR_TRENCH_2);
        break;

      case DRIVE_CLEAR_TRENCH_2:
        driveToWaypoint(pose, TRENCH_CLEAR_POS, 180.0, Phase.DRIVE_TO_SHOOT_2);
        break;

      case DRIVE_TO_SHOOT_2:
        driveToWaypoint(pose, SHOOT_POS, 120.0, Phase.SHOOT_2);
        break;

      case SHOOT_2:
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

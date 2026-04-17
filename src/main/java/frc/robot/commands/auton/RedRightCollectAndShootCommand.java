// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.ShootWithStartupCommand;
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
  private Command m_shootCommand;

  private static final Translation2d COLLECTOR_DEPLOY_POS = new Translation2d(10.69, 7.62);
  private static final Translation2d COLLECT_ALIGN_POS    = new Translation2d(8.72,  6.90);
  private static final Translation2d COLLECT_POS          = new Translation2d(8.72,  5.78);
  private static final Translation2d SWEEP_MID_POS        = new Translation2d(9.68,  5.00);
  private static final Translation2d SWEEP_POS            = new Translation2d(10.58, 5.78);
  private static final Translation2d AVOID_POS            = new Translation2d(10.78, 7.50);
  private static final Translation2d TRENCH_ENTRY_POS     = new Translation2d(13.33, 7.62);
  private static final Translation2d TRENCH_EXIT_POS      = new Translation2d(10.62, 7.62);
  private static final Translation2d TRANSIT_POS          = new Translation2d(12.88, 7.62);
  private static final Translation2d SHOOT_POS            = new Translation2d(13.33, 6.10);
  private static final double DRIVE_TOLERANCE             = 0.25;
  private static final double SHOOT_DURATION              = 2.5;

  private Phase m_phase;

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
    m_shootCommand = null;
    m_rotationController.reset();

    // DEBUG: Confirm command is initialized
    SmartDashboard.putString("RedRight/Status", "INITIALIZED");
    SmartDashboard.putNumber("RedRight/InitTime", System.currentTimeMillis() / 1000.0);
    System.out.println("RedRightCollectAndShootCommand INITIALIZED at " + m_drive.getPose());
  }

  @Override
  public void execute() {
    double t = now();
    Pose2d pose = m_drive.getPose();

    // DEBUG: Confirm execute() is being called (only log occasionally to avoid spam)
    if (m_phase == Phase.DRIVE_TO_COLLECTOR_DEPLOY) {
      System.out.println("RedRight execute() called - Phase: " + m_phase + " Pose: " + pose);
    }

    // Debug output
    SmartDashboard.putString("RedRight/Phase", m_phase.toString());
    SmartDashboard.putNumber("RedRight/PoseX", pose.getX());
    SmartDashboard.putNumber("RedRight/PoseY", pose.getY());
    SmartDashboard.putNumber("RedRight/Heading", m_drive.getHeading());

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
        if (m_shootCommand == null) {
          m_shootCommand = new ShootWithStartupCommand(m_shooter, m_drive, m_collector)
            .withTimeout(SHOOT_DURATION);
          m_shootCommand.schedule();
        }
        if (m_shootCommand.isFinished()) {
          m_shootCommand = null;
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
        driveToWaypoint(pose, SWEEP_MID_POS, 0.0, Phase.DRIVE_SWEEP_2);
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
      // Simple proportional speed control (proven to work)
      double speed = Math.min(0.5, dist * 2.0);
      double xSpeed = (dx / dist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeed = (dy / dist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      m_drive.drive(xSpeed, ySpeed, rot * DriveConstants.kMaxAngularSpeed, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // DEBUG: Log why command ended
    SmartDashboard.putString("RedRight/Status", interrupted ? "INTERRUPTED" : "FINISHED");
    SmartDashboard.putString("RedRight/EndPhase", m_phase.toString());
    System.out.println("RedRightCollectAndShootCommand ENDED - interrupted=" + interrupted + " phase=" + m_phase);

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

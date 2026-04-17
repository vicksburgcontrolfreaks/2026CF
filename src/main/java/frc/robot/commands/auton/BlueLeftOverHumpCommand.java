// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

/**
 * Blue alliance LEFT collect-and-shoot autonomous — OVER HUMP variant.
 * Mirror of Red Right - waypoints reflected across field length (X = 8.23m)
 * Returns over the hump both times instead of going under the trench.
 */
public class BlueLeftOverHumpCommand extends Command {

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

  // Mirrored waypoints: X_new = 16.46 - X_original (reflects across X = 8.23)
  private static final Translation2d COLLECTOR_DEPLOY_POS = new Translation2d(5.77,  7.5);   // was 10.69
  private static final Translation2d COLLECT_ALIGN_POS    = new Translation2d(7.06,  6.5);   // was 9.4
  private static final Translation2d COLLECT_POS          = new Translation2d(7.66,  5.5);   // was 8.8
  private static final Translation2d SWEEP_MID_POS        = new Translation2d(6.96,  4.5);   // was 9.5
  private static final Translation2d PRE_HUMP_POS         = new Translation2d(6.16,  5.7);   // was 10.3
  private static final Translation2d SHOOT_POS            = new Translation2d(2.26,  5.6);   // was 14.2

  // Second cycle waypoints (mirrored)
  private static final Translation2d COLLECT_ALIGN_POS_2  = new Translation2d(7.76,  5.15);  // was 8.7
  private static final Translation2d INTERMEDIATE_POS_2   = new Translation2d(7.66,  3.9);   // was 8.8
  private static final Translation2d COLLECT_POS_2        = new Translation2d(6.46,  3.9);   // was 10.0
  private static final Translation2d SWEEP_MID_POS_2      = new Translation2d(5.86,  4.7);   // was 10.6
  private static final Translation2d PRE_HUMP_POS_2       = new Translation2d(5.86,  5.6);   // was 10.6
  private static final double DRIVE_TOLERANCE             = 0.25;
  private static final double SMOOTH_TOLERANCE            = 0.5;
  private static final double LOOKAHEAD_DISTANCE          = 0.75;
  private static final double HUMP_SPEED                  = 0.5;
  private static final double COLLECTION_MAX_SPEED        = 0.85;
  private static final double SHOOT_DURATION              = 3.5;
  private static final int    VISION_SETTLE_MIN_TAGS      = 2;
  private static final double VISION_SETTLE_TIMEOUT       = 1.5;

  private Phase m_phase;
  private boolean m_shootingStarted;
  private double m_shootingStartTime;
  private double m_settleStartTime;
  private boolean m_hopperPopHigh;
  private double m_lastPopTime;

  public BlueLeftOverHumpCommand(DriveSubsystem drive, ShooterSubsystem shooter,
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
    m_shootingStarted = false;
    m_shootingStartTime = 0;
    m_settleStartTime = 0;
    m_hopperPopHigh = false;
    m_lastPopTime = 0;
    m_rotationController.reset();

    m_collector.runCollector(false);
  }

  @Override
  public void execute() {
    double t = now();
    Pose2d pose = m_drive.getPose();

    switch (m_phase) {

      case DRIVE_TO_COLLECTOR_DEPLOY:
        m_collector.extendHopper();  // Deploy hopper while driving
        driveToWaypointSmooth(pose, COLLECTOR_DEPLOY_POS, COLLECT_ALIGN_POS, 0.0, Phase.DRIVE_TO_COLLECT_ALIGN);
        break;

      case DRIVE_TO_COLLECT_ALIGN:
        driveToWaypointSmooth(pose, COLLECT_ALIGN_POS, COLLECT_POS, -45.0, Phase.DRIVE_COLLECT);
        break;

      case DRIVE_COLLECT:
        driveToWaypointSmooth(pose, COLLECT_POS, SWEEP_MID_POS, -90.0, Phase.DRIVE_SWEEP_MID);
        break;

      case DRIVE_SWEEP_MID:
        driveToWaypoint(pose, SWEEP_MID_POS, 135.0, Phase.DRIVE_PRE_HUMP);
        break;

      case DRIVE_PRE_HUMP:
        driveToWaypoint(pose, PRE_HUMP_POS, 135.0, Phase.DRIVE_OVER_HUMP);
        break;

      case DRIVE_OVER_HUMP:
        driveOverHump(pose, SHOOT_POS, 135.0, Phase.VISION_SETTLE);
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
        aimAndShoot(pose);
        if (m_shootingStarted && (t - m_shootingStartTime) >= SHOOT_DURATION) {
          m_shooter.StopFloor();
          m_shooter.StopIndexer();
          m_shooter.enableRPMCap();
          m_shootingStarted = false;
          m_hopperPopHigh = false;
          m_lastPopTime = 0;
          m_rotationController.reset();
          m_phase = Phase.DRIVE_BACK_OVER_HUMP;
        }
        break;

      case DRIVE_BACK_OVER_HUMP:
        driveOverHump(pose, PRE_HUMP_POS, 0.0, Phase.VISION_SETTLE_RETURN);
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
        driveToWaypointSmooth(pose, COLLECT_ALIGN_POS_2, INTERMEDIATE_POS_2, -60.0, Phase.DRIVE_INTERMEDIATE_2);
        break;

      case DRIVE_INTERMEDIATE_2:
        driveToWaypointSmooth(pose, INTERMEDIATE_POS_2, COLLECT_POS_2, -135.0, Phase.DRIVE_COLLECT_2);
        break;

      case DRIVE_COLLECT_2:
        driveToWaypointSmooth(pose, COLLECT_POS_2, SWEEP_MID_POS_2, 140.0, Phase.DRIVE_SWEEP_MID_2);
        break;

      case DRIVE_SWEEP_MID_2:
        driveToWaypoint(pose, SWEEP_MID_POS_2, 90.0, Phase.DRIVE_PRE_HUMP_2);
        break;

      case DRIVE_PRE_HUMP_2:
        driveToWaypoint(pose, PRE_HUMP_POS_2, 135.0, Phase.DRIVE_OVER_HUMP_2);
        break;

      case DRIVE_OVER_HUMP_2:
        driveOverHump(pose, SHOOT_POS, 135.0, Phase.VISION_SETTLE_2);
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

  private void driveToWaypointSmooth(Pose2d pose, Translation2d currentWaypoint,
                                      Translation2d nextWaypoint, double targetHeading,
                                      Phase nextPhase) {
    double dx = currentWaypoint.getX() - pose.getX();
    double dy = currentWaypoint.getY() - pose.getY();
    double dist = Math.sqrt(dx * dx + dy * dy);

    Translation2d targetPoint = currentWaypoint;
    if (nextWaypoint != null && dist < LOOKAHEAD_DISTANCE) {
      double blendFactor = 1.0 - (dist / LOOKAHEAD_DISTANCE);
      targetPoint = new Translation2d(
        currentWaypoint.getX() * (1 - blendFactor) + nextWaypoint.getX() * blendFactor,
        currentWaypoint.getY() * (1 - blendFactor) + nextWaypoint.getY() * blendFactor
      );
    }

    dx = targetPoint.getX() - pose.getX();
    dy = targetPoint.getY() - pose.getY();
    double targetDist = Math.sqrt(dx * dx + dy * dy);

    double rot = m_rotationController.calculate(m_drive.getHeading(), targetHeading);
    rot = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
          Math.min( AutoConstants.kRotateToTargetMaxVelocity, rot));

    if (dist < SMOOTH_TOLERANCE) {
      m_rotationController.reset();
      m_phase = nextPhase;
    } else {
      double speed = Math.min(COLLECTION_MAX_SPEED, dist * 1.5);
      double xSpeed = (dx / targetDist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeed = (dy / targetDist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      m_drive.drive(xSpeed, ySpeed, rot * DriveConstants.kMaxAngularSpeed, true);
    }
  }

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

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
import frc.robot.commands.shooter.AutoAimShootCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Blue alliance CENTER autonomous. Starts at 3.40, 4.00 facing 180°.
 * Drives to shoot position (2.55, 4.00), shoots preloaded balls, then collects
 * additional game pieces from waypoints (1.04, 7.06) and (0.38, 7.4),
 * returns to shoot position and shoots again.
 */
public class BlueCenterShootCommand extends Command {

  private enum Phase {
    DRIVE_OUT,
    SHOOT,
    START_COLLECTION,
    DRIVE_TO_COLLECT_1,
    DRIVE_TO_COLLECT_2,
    RETURN_TO_SHOOT,
    SHOOT_AGAIN,
    DONE
  }

  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final CollectorSubsystem m_collector;
  private final PIDController m_rotationController;
  private Command m_shootCommand;

  private static final Translation2d SHOOT_POS       = new Translation2d(2.55, 4.00);  // 16.46 - 13.91
  private static final Translation2d COLLECT_POS_1   = new Translation2d(1.04, 7.06);  // 16.46 - 15.42
  private static final Translation2d COLLECT_POS_2   = new Translation2d(0.38, 7.4);   // 16.46 - 16.08
  private static final double DRIVE_TOLERANCE        = 0.15;
  private static final double SHOOT_DURATION         = 3.0;
  private static final double COLLECTION_SPEED       = 0.3;  // Slower speed for collection waypoints

  private Phase m_phase;

  public BlueCenterShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
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
    m_phase = Phase.DRIVE_OUT;
    m_shootCommand = null;
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    Pose2d pose = m_drive.getPose();

    switch (m_phase) {

      case DRIVE_OUT:
        driveToWaypoint(pose, SHOOT_POS, 180.0, Phase.SHOOT);
        break;

      case SHOOT:
        if (m_shootCommand == null) {
          m_shootCommand = new AutoAimShootCommand(m_shooter, m_drive, m_collector)
            .withTimeout(SHOOT_DURATION);
          m_shootCommand.schedule();
        }
        if (m_shootCommand.isFinished()) {
          m_shootCommand = null;
          m_phase = Phase.START_COLLECTION;
        }
        break;

      case START_COLLECTION:
        m_collector.runCollector(true);
        m_collector.extendHopper();
        m_phase = Phase.DRIVE_TO_COLLECT_1;
        break;

      case DRIVE_TO_COLLECT_1:
        driveToWaypoint(pose, COLLECT_POS_1, -140.0, COLLECTION_SPEED, Phase.DRIVE_TO_COLLECT_2);
        break;

      case DRIVE_TO_COLLECT_2:
        driveToWaypoint(pose, COLLECT_POS_2, 180.0, COLLECTION_SPEED, Phase.RETURN_TO_SHOOT);
        break;

      case RETURN_TO_SHOOT:
        driveToWaypoint(pose, SHOOT_POS, 180.0, Phase.SHOOT_AGAIN);
        break;

      case SHOOT_AGAIN:
        if (m_shootCommand == null) {
          m_shootCommand = new AutoAimShootCommand(m_shooter, m_drive, m_collector)
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
    driveToWaypoint(pose, waypoint, targetHeading, 0.5, nextPhase);
  }

  private void driveToWaypoint(Pose2d pose, Translation2d waypoint, double targetHeading,
                                double maxSpeed, Phase nextPhase) {
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
      double speed = Math.min(maxSpeed, dist * 2.0);
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
}

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
 * Red alliance CENTER autonomous. Starts at ~12.09, 4.00 facing 0°.
 * Drives 1 meter away from speaker to ~13.91, 4.00, then aims and shoots preloaded balls.
 */
public class RedCenterShootCommand extends Command {

  private enum Phase {
    DRIVE_OUT,
    SHOOT,
    DONE
  }

  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final CollectorSubsystem m_collector;
  private final PIDController m_rotationController;

  private static final Translation2d SHOOT_POS   = new Translation2d(13.91, 4.00);
  private static final double DRIVE_TOLERANCE    = 0.15;
  private static final double SHOOT_DURATION     = 3.0;

  private Phase m_phase;
  private boolean m_shootingStarted;
  private double m_shootingStartTime;
  private boolean m_hopperPopHigh;
  private double m_lastPopTime;

  public RedCenterShootCommand(DriveSubsystem drive, ShooterSubsystem shooter,
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

      case DRIVE_OUT:
        driveToWaypoint(pose, SHOOT_POS, 0.0, Phase.SHOOT);
        break;

      case SHOOT:
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

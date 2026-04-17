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
 * Blue alliance two-piece autonomous: shoot at start, drive to pickup, wait, drive to second
 * shoot position, aim and shoot again.
 *
 * Sequence:
 * 1. Aim at speaker and shoot (from start pose ~3.57, 2.44)
 * 2. Deploy hopper + drive to pickup position (0.51, 0.68)
 * 3. Wait 3 seconds at pickup (collector running)
 * 4. Drive to second shoot position (2.46, 2.69), pre-spin shooter
 * 5. Aim and shoot; 2s after indexers start, set hopper to 0.07
 */
public class BlueTwoPieceAutoCommand extends Command {

  private enum Phase {
    SHOOT_FIRST,
    DRIVE_TO_PICKUP,
    WAIT_AT_PICKUP,
    DRIVE_TO_SHOOT,
    SHOOT_SECOND,
    DONE
  }

  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final CollectorSubsystem m_collector;
  private final PIDController m_rotationController;
  private Command m_shootCommand;

  private static final Translation2d PICKUP_POSITION = new Translation2d(0.61,  0.68);
  private static final Translation2d SHOOT_POSITION  = new Translation2d(2.46,  2.69);
  private static final double DRIVE_TOLERANCE       = 0.15; // meters
  private static final double FIRST_SHOOT_DURATION  = 3.0;  // seconds
  private static final double PICKUP_WAIT_DURATION  = 3.0;  // seconds
  private static final double SECOND_SHOOT_DURATION = 3.0;  // seconds

  private Phase m_phase;
  private double m_phaseStartTime;

  public BlueTwoPieceAutoCommand(DriveSubsystem drive, ShooterSubsystem shooter,
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
    m_phase = Phase.SHOOT_FIRST;
    m_phaseStartTime = now();
    m_shootCommand = null;
    m_rotationController.reset();

    // Shooter already spinning from autonomousInit() at capped RPM (3500)
    // enableFullRPM() called only when aiming begins
  }

  @Override
  public void execute() {
    double t = now();
    Pose2d pose = m_drive.getPose();

    switch (m_phase) {

      case SHOOT_FIRST:
        if (m_shootCommand == null) {
          m_shootCommand = new ShootWithStartupCommand(m_shooter, m_drive, m_collector)
            .withTimeout(FIRST_SHOOT_DURATION);
          m_shootCommand.schedule();
        }
        if (m_shootCommand.isFinished()) {
          m_shootCommand = null;
          m_collector.extendHopper();
          m_collector.runCollector(false);
          m_phase = Phase.DRIVE_TO_PICKUP;
          m_phaseStartTime = t;
        }
        break;

      case DRIVE_TO_PICKUP:
        driveToWaypoint(pose, PICKUP_POSITION, 180.0, t);
        break;

      case WAIT_AT_PICKUP:
        m_drive.drive(0, 0, 0, true);
        if ((t - m_phaseStartTime) >= PICKUP_WAIT_DURATION) {
          m_collector.stopCollector();
          // Shooter already running at cap — enableFullRPM() called in ShootWithStartupCommand
          m_phase = Phase.DRIVE_TO_SHOOT;
          m_phaseStartTime = t;
        }
        break;

      case DRIVE_TO_SHOOT:
        driveToWaypoint(pose, SHOOT_POSITION, 180.0, t);
        break;

      case SHOOT_SECOND:
        if (m_shootCommand == null) {
          m_shootCommand = new ShootWithStartupCommand(m_shooter, m_drive, m_collector)
            .withTimeout(SECOND_SHOOT_DURATION);
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

  /** Drive toward waypoint with proportional speed and heading control; transition phase on arrival. */
  private void driveToWaypoint(Pose2d pose, Translation2d waypoint, double targetHeading, double t) {
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
        if (m_phase == Phase.DRIVE_TO_PICKUP) {
          m_phase = Phase.WAIT_AT_PICKUP;
        } else if (m_phase == Phase.DRIVE_TO_SHOOT) {
          m_phase = Phase.SHOOT_SECOND;
        }
        m_phaseStartTime = t;
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

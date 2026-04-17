// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Drives back and forth between two waypoints to tune WaypointPID constants live.
 * Reads P/I/D/MaxSpeed from the "WaypointPID" NetworkTable each loop.
 *
 * Point A: (14.25, 7.0)  @ 45°
 * Point B: (14.25, 4.8)  @ 0°
 */
public class WaypointPIDTestCommand extends Command {

  private enum Phase { DRIVE_TO_A, DRIVE_TO_B }

  private final DriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final PIDController m_translationController;
  private final PIDController m_rotationController;

  private final DoubleEntry m_waypointPEntry;
  private final DoubleEntry m_waypointIEntry;
  private final DoubleEntry m_waypointDEntry;
  private final DoubleEntry m_waypointMaxSpeedEntry;
  private final DoubleEntry m_rotPEntry;
  private final DoubleEntry m_rotDEntry;
  private final DoubleEntry m_rotMaxVelEntry;

  private static final Translation2d POINT_A        = new Translation2d(14.25, 7.0);
  private static final Translation2d POINT_B        = new Translation2d(14.25, 4.8);
  private static final double        HEADING_A      = 45.0;
  private static final double        HEADING_B      = 0.0;
  private static final double        DRIVE_TOLERANCE = 0.25;
  private static final double        DEFAULT_MAX_SPEED = 0.3;

  private Phase m_phase;
  private double m_waypointMaxSpeed = DEFAULT_MAX_SPEED;

  public WaypointPIDTestCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
    m_drive = drive;
    m_shooter = shooter;

    m_translationController = new PIDController(
      AutoConstants.kWaypointP,
      AutoConstants.kWaypointI,
      AutoConstants.kWaypointD
    );
    m_translationController.setTolerance(DRIVE_TOLERANCE);

    m_rotationController = new PIDController(
      AutoConstants.kRotateToTargetP,
      AutoConstants.kRotateToTargetI,
      AutoConstants.kRotateToTargetD
    );
    m_rotationController.enableContinuousInput(-180, 180);
    m_rotationController.setTolerance(AutoConstants.kRotateToTargetTolerance);

    // Shared WaypointPID table — same entries used by RedRightCollectAndShootCommand
    var table = NetworkTableInstance.getDefault().getTable("WaypointPID");
    m_waypointPEntry        = table.getDoubleTopic("P").getEntry(AutoConstants.kWaypointP);
    m_waypointIEntry        = table.getDoubleTopic("I").getEntry(AutoConstants.kWaypointI);
    m_waypointDEntry        = table.getDoubleTopic("D").getEntry(AutoConstants.kWaypointD);
    m_waypointMaxSpeedEntry = table.getDoubleTopic("MaxSpeed").getEntry(DEFAULT_MAX_SPEED);
    m_waypointPEntry.setDefault(AutoConstants.kWaypointP);
    m_waypointIEntry.setDefault(AutoConstants.kWaypointI);
    m_waypointDEntry.setDefault(AutoConstants.kWaypointD);
    m_waypointMaxSpeedEntry.setDefault(DEFAULT_MAX_SPEED);

    m_rotPEntry      = table.getDoubleTopic("RotP").getEntry(AutoConstants.kRotateToTargetP);
    m_rotDEntry      = table.getDoubleTopic("RotD").getEntry(AutoConstants.kRotateToTargetD);
    m_rotMaxVelEntry = table.getDoubleTopic("RotMaxVel").getEntry(AutoConstants.kRotateToTargetMaxVelocity);
    m_rotPEntry.setDefault(AutoConstants.kRotateToTargetP);
    m_rotDEntry.setDefault(AutoConstants.kRotateToTargetD);
    m_rotMaxVelEntry.setDefault(AutoConstants.kRotateToTargetMaxVelocity);

    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    m_phase = Phase.DRIVE_TO_B;
    m_translationController.reset();
    m_rotationController.reset();
    m_shooter.stopShooter();
  }

  @Override
  public void execute() {
    m_translationController.setPID(
      m_waypointPEntry.get(),
      m_waypointIEntry.get(),
      m_waypointDEntry.get()
    );
    m_waypointMaxSpeed = m_waypointMaxSpeedEntry.get();
    m_rotationController.setPID(m_rotPEntry.get(), 0.0, m_rotDEntry.get());

    Pose2d pose = m_drive.getPose();

    switch (m_phase) {
      case DRIVE_TO_A:
        driveToWaypoint(pose, POINT_A, HEADING_A, Phase.DRIVE_TO_B);
        break;
      case DRIVE_TO_B:
        driveToWaypoint(pose, POINT_B, HEADING_B, Phase.DRIVE_TO_A);
        break;
    }
  }

  private void driveToWaypoint(Pose2d pose, Translation2d waypoint, double targetHeading,
                                Phase nextPhase) {
    double dx = waypoint.getX() - pose.getX();
    double dy = waypoint.getY() - pose.getY();
    double dist = Math.sqrt(dx * dx + dy * dy);

    double rotMaxVel = m_rotMaxVelEntry.get();
    double rot = m_rotationController.calculate(m_drive.getHeading(), targetHeading);
    rot = Math.max(-rotMaxVel, Math.min(rotMaxVel, rot));

    if (dist < DRIVE_TOLERANCE) {
      m_translationController.reset();
      m_rotationController.reset();
      m_phase = nextPhase;
    } else {
      double speed = MathUtil.clamp(m_translationController.calculate(0, dist), 0, m_waypointMaxSpeed);
      double xSpeed = (dx / dist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeed = (dy / dist) * speed * DriveConstants.kMaxSpeedMetersPerSecond;
      m_drive.drive(xSpeed, ySpeed, rot * DriveConstants.kMaxAngularSpeed, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false; // runs until auton period ends
  }
}

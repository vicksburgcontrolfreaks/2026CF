// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

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

import java.util.function.DoubleSupplier;

/**
 * AutoAimShootCommand - Unified shooting command for both autonomous and teleop.
 *
 * Features:
 * - Stops collector BEFORE spinning up shooter (manages high current draw)
 * - Automatically aims at speaker using PID rotation control
 * - Allows translation control via suppliers (joystick in teleop, zero in autonomous)
 * - Feeds balls only when aligned and at target RPM
 * - Works seamlessly in both autonomous routines and teleop trigger binding
 */
public class AutoAimShootCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final DriveSubsystem m_drive;
  private final CollectorSubsystem m_collector;
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final PIDController m_rotationController;
  private boolean m_feedingStarted = false;

  // Hopper popper state (automatic in autonomous)
  private boolean m_hopperPopHigh = false;
  private double m_lastPopTime = 0.0;
  private static final double HOPPER_POP_INTERVAL = 0.25;  // seconds
  private static final double HOPPER_DOWN_POSITION = 0.02;
  private static final double HOPPER_UP_POSITION = 0.19;

  /**
   * Creates a new AutoAimShootCommand.
   *
   * @param shooter ShooterSubsystem to control
   * @param drive DriveSubsystem for auto-rotation
   * @param collector CollectorSubsystem to manage (stopped before shooting)
   * @param xSpeedSupplier Forward/backward speed (joystick in teleop, () -> 0.0 in auto)
   * @param ySpeedSupplier Left/right speed (joystick in teleop, () -> 0.0 in auto)
   */
  public AutoAimShootCommand(
      ShooterSubsystem shooter,
      DriveSubsystem drive,
      CollectorSubsystem collector,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier) {
    m_shooter = shooter;
    m_drive = drive;
    m_collector = collector;
    m_xSpeedSupplier = xSpeedSupplier;
    m_ySpeedSupplier = ySpeedSupplier;

    m_rotationController = new PIDController(
        AutoConstants.kRotateToTargetP,
        AutoConstants.kRotateToTargetI,
        AutoConstants.kRotateToTargetD
    );
    m_rotationController.enableContinuousInput(-180, 180);
    m_rotationController.setTolerance(AutoConstants.kRotateToTargetTolerance);

    addRequirements(shooter, drive, collector);
  }

  /**
   * Convenience constructor for autonomous - no translation movement.
   */
  public AutoAimShootCommand(
      ShooterSubsystem shooter,
      DriveSubsystem drive,
      CollectorSubsystem collector) {
    this(shooter, drive, collector, () -> 0.0, () -> 0.0);
  }

  @Override
  public void initialize() {
    // CRITICAL: Stop collector BEFORE enabling shooter to manage current draw
    m_collector.stopCollector();

    m_feedingStarted = false;
    m_rotationController.reset();

    // Initialize hopper popper state
    m_hopperPopHigh = false;
    m_lastPopTime = 0.0;
    m_collector.setHopperPosition(HOPPER_DOWN_POSITION);

    // Enable full RPM (remove any cap)
    m_shooter.enableFullRPM();
  }

  @Override
  public void execute() {
    // Determine target speaker based on alliance
    Translation2d targetPosition;
    if (DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == Alliance.Blue) {
      targetPosition = AutoConstants.blueTarget;
    } else {
      targetPosition = AutoConstants.redTarget;
    }

    // Get current robot pose
    Pose2d currentPose = m_drive.getPose();

    // Calculate angle to target (back of robot facing speaker)
    double deltaX = targetPosition.getX() - currentPose.getX();
    double deltaY = targetPosition.getY() - currentPose.getY();
    double targetAngleDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));

    // Add 180 degrees to point the BACK of the robot at the target
    targetAngleDegrees = (targetAngleDegrees + 180) % 360;
    if (targetAngleDegrees > 180) {
      targetAngleDegrees -= 360;
    }

    // Get current heading
    double currentHeading = m_drive.getHeading();

    // Calculate rotation using PID
    double rotationSpeed = m_rotationController.calculate(currentHeading, targetAngleDegrees);

    // Clamp rotation speed to max velocity
    rotationSpeed = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
                             Math.min(AutoConstants.kRotateToTargetMaxVelocity, rotationSpeed));

    // Drive with translation input (from suppliers) and rotation PID
    m_drive.drive(
        m_xSpeedSupplier.getAsDouble(),
        m_ySpeedSupplier.getAsDouble(),
        rotationSpeed * DriveConstants.kMaxAngularSpeed,
        true
    );

    // Check if we should feed balls: aligned AND at target RPM
    boolean isAligned = m_rotationController.atSetpoint();
    boolean isAtRPM = m_shooter.isReadyToFeed();
    boolean shouldFeed = isAligned && isAtRPM;

    if (shouldFeed && !m_feedingStarted) {
      // Start feeding - run indexer and floor motors
      m_shooter.runIndexer(false);
      m_shooter.runFloor(false);
      m_lastPopTime = now();  // Initialize hopper pop timer when feeding starts
      m_feedingStarted = true;
    }

    // Automatic hopper popper during shooting (in autonomous)
    if (m_feedingStarted) {
      double currentTime = now();
      if (currentTime - m_lastPopTime >= HOPPER_POP_INTERVAL) {
        m_hopperPopHigh = !m_hopperPopHigh;
        m_collector.setHopperPosition(m_hopperPopHigh ? HOPPER_UP_POSITION : HOPPER_DOWN_POSITION);
        m_lastPopTime = currentTime;
      }
    }

    // Note: Once feeding starts, DON'T stop it due to minor alignment fluctuations
    // This prevents jitter from PID micro-corrections
  }

  private static double now() {
    return System.currentTimeMillis() / 1000.0;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop feeding motors
    m_shooter.StopFloor();
    m_shooter.StopIndexer();
    m_collector.stopCollector();

    // Re-enable RPM cap to drop back to pre-spin RPM (energy saving)
    m_shooter.enableRPMCap();

    // Reset rotation PID
    m_rotationController.reset();

    // Stop drive
    m_drive.stop();
  }

  @Override
  public boolean isFinished() {
    // Command runs until canceled (trigger released in teleop, or timeout in auto)
    return false;
  }
}

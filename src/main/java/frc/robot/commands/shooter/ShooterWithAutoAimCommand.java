// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
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
 * ShooterWithAutoAimCommand - Controls indexer and floor feed motors while automatically
 * rotating the robot to face the speaker target.
 *
 * The shooter wheels are already spinning from autonomousInit().
 * This command:
 * 1. Activates the indexer and floor to feed game pieces
 * 2. Automatically rotates the robot to face the alliance's speaker
 * 3. Allows the driver to maintain translational control (forward/backward/strafe)
 *
 * Bound to right trigger - runs while button is held.
 */
public class ShooterWithAutoAimCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final DriveSubsystem m_swerveDrive;
  private final CollectorSubsystem m_collector;
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final PIDController m_rotationController;
  private final LinearFilter m_angleOffsetFilter;
  private boolean m_feedingStarted = false;

  /**
   * Creates a new ShooterWithAutoAimCommand.
   *
   * @param shooter ShooterSubsystem to control
   * @param swerveDrive DriveSubsystem for auto-rotation
   * @param xSpeedSupplier Supplier for forward/backward speed from joystick
   * @param ySpeedSupplier Supplier for left/right speed from joystick
   */
  public ShooterWithAutoAimCommand(
      ShooterSubsystem shooter,
      DriveSubsystem swerveDrive,
      CollectorSubsystem collector,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier) {
    m_shooter = shooter;
    m_swerveDrive = swerveDrive;
    m_collector = collector;
    m_xSpeedSupplier = xSpeedSupplier;
    m_ySpeedSupplier = ySpeedSupplier;

    m_rotationController = new PIDController(
        AutoConstants.kRotateToTargetP,
        AutoConstants.kRotateToTargetI,
        AutoConstants.kRotateToTargetD
    );

    // Enable continuous input for angle wrapping (-180 to 180 degrees)
    m_rotationController.enableContinuousInput(-180, 180);
    m_rotationController.setTolerance(AutoConstants.kRotateToTargetTolerance);

    // Low-pass filter for angle compensation offset (0.30s time constant at 50Hz)
    // Smooths velocity spikes from carpet bumps so the PID setpoint changes gradually
    m_angleOffsetFilter = LinearFilter.singlePoleIIR(0.30, 0.02);

    addRequirements(shooter, swerveDrive, collector);
  }

  @Override
  public void initialize() {
    // Reset the rotation PID controller and angle filter
    m_rotationController.reset();
    m_angleOffsetFilter.reset();

    // Reset feeding state
    m_feedingStarted = false;

    // Disable RPM cap so shooter can ramp up to full calculated RPM
    // This allows the shooter to go from pre-spin (2000 RPM) to full power (e.g. 4300 RPM)
    m_shooter.enableFullRPM();

    // Shooter wheels are already spinning from teleopInit()
    // Don't start feeding yet - wait for alignment in execute()
  }

  @Override
  public void execute() {
    // Determine target position based on alliance
    Translation2d targetPosition;
    if (DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == Alliance.Blue) {
      targetPosition = AutoConstants.blueTarget;
    } else {
      targetPosition = AutoConstants.redTarget;
    }

    // Get current robot pose
    Pose2d currentPose = m_swerveDrive.getPose();

    // Calculate the angle to the target
    double deltaX = targetPosition.getX() - currentPose.getX();
    double deltaY = targetPosition.getY() - currentPose.getY();
    double targetAngleRadians = Math.atan2(deltaY, deltaX);

    // Apply velocity compensation — filter smooths rapid velocity fluctuations
    double rawOffset = m_shooter.getVelocityCompensatedAngleOffset();
    targetAngleRadians += m_angleOffsetFilter.calculate(rawOffset);

    // Convert to degrees
    double targetAngleDegrees = Math.toDegrees(targetAngleRadians);

    // Add 180 degrees to point the BACK of the robot at the target (shooter is at the back)
    targetAngleDegrees = (targetAngleDegrees + 180) % 360;
    if (targetAngleDegrees > 180) {
      targetAngleDegrees -= 360;
    }

    // Get current heading
    double currentHeading = m_swerveDrive.getHeading();

    // Calculate rotation using PID
    double rotationSpeed = m_rotationController.calculate(currentHeading, targetAngleDegrees);

    // Clamp rotation speed to max velocity
    rotationSpeed = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
                             Math.min(AutoConstants.kRotateToTargetMaxVelocity, rotationSpeed));

    // Once feeding has started, lock wheels in X formation to resist bumps
    // Before that, drive normally with translational input and rotation PID
    if (m_feedingStarted) {
      m_swerveDrive.setX();
    } else {
      m_swerveDrive.drive(
          m_xSpeedSupplier.getAsDouble(),
          m_ySpeedSupplier.getAsDouble(),
          rotationSpeed * DriveConstants.kMaxAngularSpeed,
          true
      );
    }

    // Check if we should feed balls: aligned (shooter is always pre-spinning, no RPM wait needed)
    boolean shouldFeed = m_rotationController.atSetpoint();

    if (shouldFeed && !m_feedingStarted) {
      // Start feeding
      m_shooter.runIndexer(false);
      m_shooter.runFloor(false);
      m_collector.runCollector(false);
      m_feedingStarted = true;
    }
    // Note: Once feeding starts, DON'T stop it due to minor alignment fluctuations
    // The feeding will continue until the command ends (trigger released)
  }

  @Override
  public void end(boolean interrupted) {
    // Stop only the feed motors when button is released
    // Shooter wheels continue spinning
    m_shooter.StopFloor();
    m_shooter.StopIndexer();
    m_collector.stopCollector();

    // Re-enable RPM cap to drop back to pre-spin RPM (energy saving)
    // Rate limiting will prevent the 149A regenerative braking spike
    m_shooter.enableRPMCap();

    // IMPORTANT: Reset rotation PID to prevent residual commands
    m_rotationController.reset();
    m_angleOffsetFilter.reset();

    // Stop the drive subsystem
    m_swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    // Command runs while button is held
    return false;
  }
}

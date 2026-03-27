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
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;
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
 * 4. Left trigger on driver controller overrides auto-aim for manual rotation control
 *
 * Bound to right trigger - runs while button is held.
 */
public class ShooterWithAutoAimCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final DriveSubsystem m_swerveDrive;
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final BooleanSupplier m_manualRotationOverride;
  private final PIDController m_rotationController;

  /**
   * Creates a new ShooterWithAutoAimCommand.
   *
   * @param shooter ShooterSubsystem to control
   * @param swerveDrive DriveSubsystem for auto-rotation
   * @param xSpeedSupplier Supplier for forward/backward speed from joystick
   * @param ySpeedSupplier Supplier for left/right speed from joystick
   * @param rotationSupplier Supplier for manual rotation speed from joystick
   * @param manualRotationOverride Supplier that returns true when manual rotation is desired (left trigger pressed)
   */
  public ShooterWithAutoAimCommand(
      ShooterSubsystem shooter,
      DriveSubsystem swerveDrive,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier manualRotationOverride) {
    m_shooter = shooter;
    m_swerveDrive = swerveDrive;
    m_xSpeedSupplier = xSpeedSupplier;
    m_ySpeedSupplier = ySpeedSupplier;
    m_rotationSupplier = rotationSupplier;
    m_manualRotationOverride = manualRotationOverride;

    m_rotationController = new PIDController(
        AutoConstants.kRotateToTargetP,
        AutoConstants.kRotateToTargetI,
        AutoConstants.kRotateToTargetD
    );

    // Enable continuous input for angle wrapping (-180 to 180 degrees)
    m_rotationController.enableContinuousInput(-180, 180);
    m_rotationController.setTolerance(AutoConstants.kRotateToTargetTolerance);

    addRequirements(shooter, swerveDrive);
  }

  @Override
  public void initialize() {
    // Reset the rotation PID controller
    m_rotationController.reset();

    // Shooter wheels are already spinning from autonomousInit()
    // Just activate the indexer and floor to feed game pieces
    m_shooter.runIndexer(false);
    m_shooter.runFloor(false);
  }

  @Override
  public void execute() {
    double rotationSpeed;

    // Check if manual rotation override is active (left trigger pressed)
    if (m_manualRotationOverride.getAsBoolean()) {
      // Use manual rotation from right stick
      rotationSpeed = m_rotationSupplier.getAsDouble();
    } else {
      // Use auto-aim rotation
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
      double targetAngleDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));

      // Get current heading
      double currentHeading = m_swerveDrive.getHeading();

      // Calculate rotation using PID
      rotationSpeed = m_rotationController.calculate(currentHeading, targetAngleDegrees);

      // Clamp rotation speed to max velocity
      rotationSpeed = Math.max(-AutoConstants.kRotateToTargetMaxVelocity,
                               Math.min(AutoConstants.kRotateToTargetMaxVelocity, rotationSpeed));

      // Scale to max angular speed
      rotationSpeed = rotationSpeed * DriveConstants.kMaxAngularSpeed;
    }

    // Apply drive command with driver's translational input and rotation (auto or manual)
    m_swerveDrive.drive(
        m_xSpeedSupplier.getAsDouble(),
        m_ySpeedSupplier.getAsDouble(),
        rotationSpeed,
        true
    );

    // Continue feeding - motors stay running
  }

  @Override
  public void end(boolean interrupted) {
    // Stop only the feed motors when button is released
    // Shooter wheels continue spinning
    m_shooter.StopFloor();
    m_shooter.StopIndexer();

    // Stop the drive subsystem
    m_swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    // Command runs while button is held
    return false;
  }
}

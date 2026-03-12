// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

/**
 * ShooterCalibrationCommand - Continuously calculates and updates shooter RPM
 * based on real-time vision distance to the hub during teleop.
 *
 * This command:
 * - Runs continuously as the shooter's default command
 * - Uses PhotonVision multi-camera pose estimation and fusion
 * - Calculates distance to alliance-specific hub/speaker
 * - Applies RPM lookup table interpolation from ShooterConstants
 * - Updates shooter velocity in real-time for maximum accuracy
 */
public class ShooterCalibrationCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final PhotonVisionSubsystem m_vision;

  /**
   * Creates a new ShooterCalibrationCommand.
   *
   * @param shooter ShooterSubsystem to control
   * @param vision PhotonVisionSubsystem for distance calculation
   */
  public ShooterCalibrationCommand(ShooterSubsystem shooter, PhotonVisionSubsystem vision) {
    m_shooter = shooter;
    m_vision = vision;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // No initialization needed - command runs continuously
  }

  @Override
  public void execute() {
    // Update target RPM calculation WITHOUT spinning motors
    // PhotonVisionSubsystem.periodic() handles:
    // - Multi-camera AprilTag detection (4 cameras for 360° coverage)
    // - Multi-tag PNP pose estimation for each camera
    // - Pose fusion across all cameras for improved accuracy
    // - Continuous pose updates to drive subsystem odometry
    //
    // getDistanceToSpeaker() uses the fused pose to calculate
    // distance to the alliance-specific hub/speaker target
    //
    // updateTargetRPM() calculates optimal RPM using lookup table interpolation
    // but does NOT command the motors - they remain idle until activated
    m_shooter.updateTargetRPM(m_vision);
  }

  @Override
  public void end(boolean interrupted) {
    // This is a default command - it never ends
    // Calculations continue running in the background
  }

  @Override
  public boolean isFinished() {
    // Never finish - runs continuously during teleop
    return false;
  }
}

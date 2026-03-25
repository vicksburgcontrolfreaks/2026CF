// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

/**
 * AdvancedShooterCommand - Combined ShooterCalibrationCommand and ShooterSequenceCommand.
 *
 * Uses linear regression to calculate optimal RPM based on vision distance,
 * then runs the complete shooter sequence (shooter + floor + indexer).
 *
 * The ShooterSubsystem.periodic() method continuously calculates the target RPM
 * based on vision distance. This command simply activates the shooter motors
 * to use the pre-calculated RPM.
 *
 * Replaces both the old ShooterSequenceCommand and ShooterCalibrationCommand.
 * Bound to right trigger - runs while button is held.
 */
public class ShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private boolean m_feedingStarted = false;

  /**
   * Creates a new AdvancedShooterCommand.
   *
   * @param shooter ShooterSubsystem to control
   * @param vision PhotonVisionSubsystem (unused - kept for compatibility, subsystem handles vision internally)
   */
  public ShooterCommand(ShooterSubsystem shooter, PhotonVisionSubsystem vision) {
    m_shooter = shooter;
    // vision parameter kept for backwards compatibility but not used
    // ShooterSubsystem.periodic() handles continuous vision-based RPM calculation
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_feedingStarted = false;
    // Activate shooter wheels - wait for them to reach speed before feeding
    m_shooter.activateShooter();
  }

  @Override
  public void execute() {
    // Once shooter reaches target velocity, start feeding game pieces
    if (!m_feedingStarted && m_shooter.isAtTargetVelocity(100)) {
      m_shooter.runIndexer(false);
      m_shooter.runFloor(false);
      m_feedingStarted = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all motors when button is released
    m_shooter.stopShooter();
    m_shooter.StopFloor();
    m_shooter.StopIndexer();
  }

  @Override
  public boolean isFinished() {
    // Command runs while button is held
    return false;
  }
}

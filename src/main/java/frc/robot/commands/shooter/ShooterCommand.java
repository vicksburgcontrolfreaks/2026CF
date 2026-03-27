// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

/**
 * ShooterCommand - Controls indexer and floor feed motors.
 *
 * The shooter wheels are now activated at the beginning of autonomous mode
 * and stay spinning throughout the match for instant firing capability.
 *
 * This command only controls the indexer and floor motors to feed game pieces
 * into the already-spinning shooter wheels.
 *
 * Bound to right trigger - runs while button is held.
 */
public class ShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;

  /**
   * Creates a new ShooterCommand.
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
    // Shooter wheels are already spinning from autonomousInit()
    // Just activate the indexer and floor to feed game pieces
    m_shooter.runIndexer(false);
    m_shooter.runFloor(false);
  }

  @Override
  public void execute() {
    // Continue feeding - motors stay running
  }

  @Override
  public void end(boolean interrupted) {
    // Stop only the feed motors when button is released
    // Shooter wheels continue spinning
    m_shooter.StopFloor();
    m_shooter.StopIndexer();
  }

  @Override
  public boolean isFinished() {
    // Command runs while button is held
    return false;
  }
}

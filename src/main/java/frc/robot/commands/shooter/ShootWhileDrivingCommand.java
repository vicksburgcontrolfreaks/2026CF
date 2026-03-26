// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.DriveAndAlignCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Command that manages shooting while driving.
 * Delegates driving/alignment to DriveAndAlignCommand and manages shooter/indexer.
 *
 * Behavior:
 * - Shooter spins up immediately and maintains RPM based on distance
 * - Driver controls translation, robot automatically maintains alignment
 * - Indexer ONLY feeds when both:
 *   1. Shooter is at target RPM
 *   2. Robot is aligned to speaker within tolerance
 */
public class ShootWhileDrivingCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final DriveAndAlignCommand m_driveAndAlignCommand;

  private boolean m_isFeedingBalls = false;

  /**
   * Creates a new ShootWhileDrivingCommand.
   *
   * @param shooterSubsystem The shooter subsystem
   * @param driveAndAlignCommand The drive and align command (already configured)
   */
  public ShootWhileDrivingCommand(
      ShooterSubsystem shooterSubsystem,
      DriveAndAlignCommand driveAndAlignCommand) {
    m_shooterSubsystem = shooterSubsystem;
    m_driveAndAlignCommand = driveAndAlignCommand;

    // This command requires the shooter, drive is handled by composition
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    // Start shooter (dynamic RPM will adjust automatically)
    m_shooterSubsystem.activateShooter();

    // Start drive and align command
    m_driveAndAlignCommand.schedule();

    m_isFeedingBalls = false;
  }

  @Override
  public void execute() {
    // Check if we should feed balls
    boolean shouldFeed = m_shooterSubsystem.isReadyToFeed() &&
                         m_driveAndAlignCommand.isAligned();

    if (shouldFeed && !m_isFeedingBalls) {
      // Start feeding
      m_shooterSubsystem.runIndexer(false);
      m_shooterSubsystem.runFloor(false);
      m_isFeedingBalls = true;
    } else if (!shouldFeed && m_isFeedingBalls) {
      // Stop feeding (not aligned or shooter not at RPM)
      m_shooterSubsystem.StopIndexer();
      m_shooterSubsystem.StopFloor();
      m_isFeedingBalls = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop everything
    m_shooterSubsystem.stopShooter();
    m_shooterSubsystem.StopIndexer();
    m_shooterSubsystem.StopFloor();

    // Cancel the drive command
    m_driveAndAlignCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    // Never finishes - runs until interrupted
    return false;
  }
}

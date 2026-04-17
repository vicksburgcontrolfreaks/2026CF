// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;

/**
 * Command that pops the hopper up and down at a regular interval.
 * Useful for helping balls feed into the shooter during manual operation.
 *
 * NOTE: This command does NOT require the collector subsystem so it can run
 * in parallel with other commands (like auto aim shooting).
 */
public class HopperPopCommand extends Command {
  private final CollectorSubsystem m_collector;
  private final Timer m_popTimer;
  private boolean m_hopperHigh = false;
  private static final double POP_INTERVAL = 0.25; // seconds
  private static final double HOPPER_DOWN_POSITION = 0.02;
  private static final double HOPPER_UP_POSITION = 0.19;

  public HopperPopCommand(CollectorSubsystem collector) {
    m_collector = collector;
    m_popTimer = new Timer();
    // Do NOT add requirements - this allows running in parallel with other commands
  }

  @Override
  public void initialize() {
    // Start with hopper in up position
    m_hopperHigh = true;
    m_collector.setHopperPosition(HOPPER_UP_POSITION);
    m_popTimer.reset();
    m_popTimer.start();
  }

  @Override
  public void execute() {
    // Pop hopper every POP_INTERVAL seconds
    if (m_popTimer.advanceIfElapsed(POP_INTERVAL)) {
      m_hopperHigh = !m_hopperHigh;
      m_collector.setHopperPosition(m_hopperHigh ? HOPPER_UP_POSITION : HOPPER_DOWN_POSITION);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_popTimer.stop();
    // Leave hopper at current position when command ends
  }

  @Override
  public boolean isFinished() {
    // Command runs continuously while button is held
    return false;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterSequenceCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final Timer m_timer;
  private boolean m_indexerStarted;

  public ShooterSequenceCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
    m_timer = new Timer();
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_indexerStarted = false;
    // Start shooter motors immediately
    m_shooter.runShooter();
  }

  @Override
  public void execute() {
    // After 0.5 seconds, start the indexer and floor motors
    if (!m_indexerStarted && m_timer.hasElapsed(0.5)) {
      m_shooter.runIndexer();
      m_shooter.runFloor(false);
      m_indexerStarted = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

public class ShooterSequenceCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final PhotonVisionSubsystem m_vision;
  private final Timer m_timer;
  private boolean m_indexerStarted;

  public ShooterSequenceCommand(ShooterSubsystem shooter, PhotonVisionSubsystem vision) {
    m_shooter = shooter;
    m_vision = vision;
    m_timer = new Timer();
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_indexerStarted = false;
    // Start shooter motors immediately with dynamic RPM based on distance
    m_shooter.runShooter(m_vision);
  }

  @Override
  public void execute() {
    if (!m_indexerStarted && m_timer.hasElapsed(1.5)) {
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
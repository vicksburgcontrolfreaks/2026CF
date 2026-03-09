// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterSequenceCommand extends Command {
  private final ShooterSubsystem m_shooter;

  public ShooterSequenceCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Start floor and indexer immediately
    m_shooter.runIndexer(false, false);
    m_shooter.runFloor(false);
  }

  @Override
  public void execute() {
    // Floor and indexer run continuously during the command
  }

  @Override
  public void end(boolean interrupted) {
    // Stop floor and indexer, but shooter keeps running via default command
    m_shooter.StopFloor();
    m_shooter.StopIndexer();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
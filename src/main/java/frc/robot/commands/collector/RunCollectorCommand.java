// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;

public class RunCollectorCommand extends Command {
  private final CollectorSubsystem m_collector;
  private boolean m_reversed;

  public RunCollectorCommand(CollectorSubsystem collector, boolean reversed) {
    m_collector = collector;
    m_reversed = reversed;
    addRequirements(collector);
  }

  @Override
  public void initialize() {
    m_collector.runCollector(m_reversed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    new StopCollectorCommand(m_collector);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

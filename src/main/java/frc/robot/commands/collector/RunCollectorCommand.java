// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;

public class RunCollectorCommand extends Command {
  private final CollectorSubsystem m_collector;

  public RunCollectorCommand(CollectorSubsystem collector) {
    m_collector = collector;
    addRequirements(collector);
  }

  @Override
  public void initialize() {
    m_collector.runCollector();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_collector.isHopperAtPosition(CollectorConstants.kHopperDeployedPosition);
  }
}

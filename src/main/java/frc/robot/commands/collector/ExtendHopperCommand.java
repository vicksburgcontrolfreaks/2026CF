// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CollectorConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;

public class ExtendHopperCommand extends Command {
  private final CollectorSubsystem m_collector;
  private final double m_tolerance;

  public ExtendHopperCommand(CollectorSubsystem collector) {
    this(collector, 0.05); // Default tolerance of 0.01 rotations
  }

  public ExtendHopperCommand(CollectorSubsystem collector, double tolerance) {
    m_collector = collector;
    m_tolerance = tolerance;
    addRequirements(collector);
  }

  @Override
  public void initialize() {
    m_collector.extendHopper();
  }

  @Override
  public void execute() {
    // Position control is handled by the motor controller's PID
  }

  @Override
  public void end(boolean interrupted) {
    // Hopper holds position automatically
  }

  @Override
  public boolean isFinished() {
    // Command finishes when hopper reaches extended position within tolerance
    double currentPosition = m_collector.getHopperPosition();
    return Math.abs(currentPosition - CollectorConstants.kDownPosition) < m_tolerance;
  }
}

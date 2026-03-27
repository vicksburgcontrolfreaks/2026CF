// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;

/**
 * Command that extends the hopper to a halfway position between up and down.
 */
public class ExtendHopperHalfwayCommand extends Command {
  private final CollectorSubsystem m_collector;
  private final double m_tolerance;
  private double m_halfwayPosition;

  public ExtendHopperHalfwayCommand(CollectorSubsystem collector) {
    this(collector, 0.05); // Default tolerance
  }

  public ExtendHopperHalfwayCommand(CollectorSubsystem collector, double tolerance) {
    m_collector = collector;
    m_tolerance = tolerance;
    addRequirements(collector);
  }

  @Override
  public void initialize() {
    // Calculate halfway position between up and down
    m_halfwayPosition = (m_collector.getUpPosition() + m_collector.getDownPosition()) / 2.0;

    // Set the hopper to halfway position
    m_collector.setHopperPosition(m_halfwayPosition);
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
    // Command finishes when hopper reaches halfway position within tolerance
    double currentPosition = m_collector.getHopperPosition();
    return Math.abs(currentPosition - m_halfwayPosition) < m_tolerance;
  }
}

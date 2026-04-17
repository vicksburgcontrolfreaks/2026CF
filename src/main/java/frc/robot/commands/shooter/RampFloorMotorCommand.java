// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Command that ramps the floor motor from 0% to a target duty cycle over a specified time period.
 */
public class RampFloorMotorCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final double m_targetDutyCycle;
  private final double m_rampTime;
  private double m_startTime;

  /**
   * Creates a new RampFloorMotorCommand.
   *
   * @param shooterSubsystem The shooter subsystem
   * @param targetDutyCycle Target duty cycle (0.0 to 1.0)
   * @param rampTime Time to ramp up in seconds
   */
  public RampFloorMotorCommand(
      ShooterSubsystem shooterSubsystem,
      double targetDutyCycle,
      double rampTime) {
    m_shooterSubsystem = shooterSubsystem;
    m_targetDutyCycle = targetDutyCycle;
    m_rampTime = rampTime;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double elapsed = Timer.getFPGATimestamp() - m_startTime;

    // Calculate current duty cycle based on elapsed time
    // Ramps linearly from 0 to target over rampTime seconds
    double dutyCycle = Math.min(m_targetDutyCycle, (elapsed / m_rampTime) * m_targetDutyCycle);

    m_shooterSubsystem.runFloorDutyCycle(dutyCycle);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.StopFloor();
  }

  @Override
  public boolean isFinished() {
    return false; // Run continuously until interrupted (button released)
  }
}

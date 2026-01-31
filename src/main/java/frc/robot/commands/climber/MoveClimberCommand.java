// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

/**
 * Command to move the climber up or down based on joystick/controller input.
 * The command runs continuously while held and stops when released.
 */
public class MoveClimberCommand extends Command {
  private final Climber m_climber;
  private final DoubleSupplier m_speedSupplier;

  /**
   * Creates a new MoveClimberCommand.
   *
   * @param climber The climber subsystem
   * @param speedSupplier A supplier that provides the desired speed (-1.0 to 1.0)
   *                      Positive = up, Negative = down
   */
  public MoveClimberCommand(Climber climber, DoubleSupplier speedSupplier) {
    m_climber = climber;
    m_speedSupplier = speedSupplier;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    // Nothing to initialize
  }

  @Override
  public void execute() {
    // Get the speed from the supplier and move the climber
    double speed = m_speedSupplier.getAsDouble();
    m_climber.move(speed);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the climber when the command ends
    m_climber.stop();
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted (button released)
    return false;
  }
}

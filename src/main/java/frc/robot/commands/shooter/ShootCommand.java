// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterAdjustments;

public class ShootCommand extends Command {
  private final ShooterAdjustments m_shooter;

  /**
   * Creates a new ShootCommand.
   * Top shooter runs at 80% immediately.
   * Front shooter ramps from 20% to 80%.
   * @param shooter The shooter subsystem to use
   */
  public ShootCommand(ShooterAdjustments shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Start the shooter motors with automatic ramping
    m_shooter.startShooter();
  }

  @Override
  public void execute() {
    // The periodic() method in ShooterAdjustments handles the ramping
    // Nothing needed here - just keep the command running
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the shooter motors when the command ends
    m_shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    // This command will run until interrupted (button released)
    return false;
  }
}

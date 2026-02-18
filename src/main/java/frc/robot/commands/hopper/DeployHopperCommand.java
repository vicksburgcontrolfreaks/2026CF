// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.HopperSubsystem;

public class DeployHopperCommand extends Command {

  private final HopperSubsystem m_hopper;
  private final boolean m_deploying;

  public DeployHopperCommand(HopperSubsystem hopper, boolean deploying) {
    m_hopper = hopper;
    m_deploying = deploying;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_deploying) {
      m_hopper.deploy();
    } else {
      m_hopper.retract();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

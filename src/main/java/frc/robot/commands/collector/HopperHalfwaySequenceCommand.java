// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.collector.CollectorSubsystem;

/**
 * Command that deploys the hopper halfway, waits, then retracts it back up.
 */
public class HopperHalfwaySequenceCommand extends SequentialCommandGroup {
  public HopperHalfwaySequenceCommand(CollectorSubsystem collector) {
    addCommands(
      // Deploy hopper halfway (between up and down positions)
      new ExtendHopperHalfwayCommand(collector),
      // Wait briefly at halfway position
      new WaitCommand(0.5),
      // Retract hopper back up
      new RetractHopperCommand(collector)
    );
  }
}

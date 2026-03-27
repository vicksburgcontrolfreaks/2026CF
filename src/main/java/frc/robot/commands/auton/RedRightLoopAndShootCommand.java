// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.collector.ExtendHopperCommand;
import frc.robot.commands.collector.RunCollectorCommand;
import frc.robot.commands.collector.StopCollectorCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

/**
 * RedRightLoopAndShootCommand - Autonomous routine for the red right side.
 *
 * This command follows the RedRightLoopAndShoot Choreo trajectory and executes
 * commands at specific event markers:
 * - DeployHopper: Extends the hopper
 * - RunCollector: Starts the collector
 * - StopCollector: Stops the collector
 * - Shoot: Activates the shooter (7 second timeout)
 */
public class RedRightLoopAndShootCommand extends Command {
  private final AutoRoutine m_autoRoutine;
  private final CollectorSubsystem m_collector;
  private final ShooterSubsystem m_shooter;
  private final PhotonVisionSubsystem m_vision;

  /**
   * Creates a new RedRightLoopAndShootCommand.
   *
   * @param autoFactory The Choreo AutoFactory for creating the routine
   * @param collector The collector subsystem
   * @param shooter The shooter subsystem
   * @param vision The vision subsystem
   */
  public RedRightLoopAndShootCommand(
      AutoFactory autoFactory,
      CollectorSubsystem collector,
      ShooterSubsystem shooter,
      PhotonVisionSubsystem vision) {

    m_collector = collector;
    m_shooter = shooter;
    m_vision = vision;

    // Create the AutoRoutine
    m_autoRoutine = autoFactory.newRoutine("RedRightLoopAndShootRoutine");

    // Get the trajectory using the generated ChoreoTraj constant
    AutoTrajectory trajectory = ChoreoTraj.RedRightLoopAndShoot.asAutoTraj(m_autoRoutine);

    // Bind commands to event markers
    trajectory.atTime("DeployHopper").onTrue(
      new ExtendHopperCommand(m_collector)
    );

    trajectory.atTime("RunCollector").onTrue(
      new RunCollectorCommand(m_collector, false)
    );

    trajectory.atTime("StopCollector").onTrue(
      new StopCollectorCommand(m_collector)
    );

    trajectory.atTime("Shoot").whileTrue(
      new ShooterCommand(m_shooter, m_vision).withTimeout(7)
    );

    // Trigger the trajectory to run when the routine is active
    m_autoRoutine.active().whileTrue(trajectory.cmd());
  }

  /**
   * Gets the command to run this autonomous routine.
   *
   * @return The autonomous command
   */
  public Command getCommand() {
    return m_autoRoutine.cmd();
  }
}

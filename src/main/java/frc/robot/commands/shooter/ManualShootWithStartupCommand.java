package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Manual shoot command (left trigger) - no auto-aim, no hopper popper
 * Just runs shooter wheels, indexer, floor motor, and collector
 *
 * Sequence:
 * 1. Staggered startup (0.9s)
 * 2. Wait for RPM
 * 3. Run feeding motors
 * 4. Shutdown when complete
 */
public class ManualShootWithStartupCommand extends SequentialCommandGroup {

  public ManualShootWithStartupCommand(ShooterSubsystem shooter, CollectorSubsystem collector) {
    addCommands(
      // Phase 1: Staggered startup (0.9s)
      new StaggeredShooterStartupCommand(shooter),

      // Phase 2: Wait for RPM to reach target
      new WaitUntilCommand(shooter::isReadyToFeed),

      // Phase 3: Run feeding motors (runs until trigger released)
      Commands.run(() -> {
        shooter.runIndexer(false);
        shooter.runFloor(false);
        collector.runCollector(false);
      }, shooter, collector)
        .finallyDo(() -> {
          // Phase 4: Stop feeding motors and shutdown shooter
          shooter.StopIndexer();
          shooter.StopFloor();
          collector.stopCollector();
          shooter.stopShooter();
        })
    );
  }
}

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Unified shooting command that handles complete shoot cycle:
 * 1. Staggered startup (0.9s) IN PARALLEL with auto-aim
 * 2. Once RPM reached → switch to 60A and start feeding
 * 3. Shutdown (coast) when complete
 *
 * Use this command for ALL shooting - teleop triggers and autonomous routines.
 * Ensures consistent startup/shutdown behavior across all modes.
 */
public class ShootWithStartupCommand extends SequentialCommandGroup {

  /**
   * Create shoot command with auto-aim (for teleop right trigger)
   * Translation controlled by joystick suppliers
   */
  public ShootWithStartupCommand(
      ShooterSubsystem shooter,
      DriveSubsystem drive,
      CollectorSubsystem collector,
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed) {

    addCommands(
      // Start staggered motor startup (40A) and auto-aim/shoot in parallel
      // The startup happens in the background while we aim and shoot
    
        new StaggeredSHooterSequentialCommand(shooter, drive, collector, xSpeed, ySpeed)
      
      .finallyDo(() -> {
        // Shutdown - coast motors when shooting complete
        shooter.stopShooter();
      })
    );
  }

  /**
   * Create shoot command with auto-aim for autonomous (no joystick input)
   * Translation locked at zero
   */
  public ShootWithStartupCommand(
      ShooterSubsystem shooter,
      DriveSubsystem drive,
      CollectorSubsystem collector) {

    this(shooter, drive, collector, () -> 0.0, () -> 0.0);
  }

  // Note: Use .withTimeout() from Command base class for autonomous
}

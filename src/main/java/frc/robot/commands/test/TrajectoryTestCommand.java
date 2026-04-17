package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Manual trajectory testing command.
 *
 * Activates shooter with test RPM (controlled via NetworkTables) and
 * runs feed motors when triggered. Used for systematic trajectory testing
 * with different hardware angles and RPM combinations.
 *
 * Test procedure:
 * 1. Adjust physical trajectory angle (hardware, in 2-degree increments)
 * 2. Set trajectory angle value in Shuffleboard (Shooter/TrajectoryTest/Trajectory Angle)
 * 3. Set test RPM in Shuffleboard (Shooter/TrajectoryTest/Test RPM)
 * 4. Enable test mode (Shooter/TrajectoryTest/Test Mode Enabled = true)
 * 5. Hold button to spin up and feed
 * 6. Record results (distance, success, notes)
 * 7. Adjust RPM and repeat
 * 8. Adjust trajectory angle and repeat
 *
 * Bind this command to a button on mechanism controller for testing.
 */
public class TrajectoryTestCommand extends Command {
  private final ShooterSubsystem m_shooter;

  public TrajectoryTestCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Activate shooter with test RPM
    m_shooter.activateShooter();
    m_shooter.enableFullRPM();
  }

  @Override
  public void execute() {
    // Feed balls while button is held and shooter is at RPM
    if (m_shooter.isReadyToFeed()) {
      m_shooter.runIndexer(false);
      m_shooter.runFloor(false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop feeding, but keep shooter spinning if in test mode
    m_shooter.StopFloor();
    m_shooter.StopIndexer();

    // If test mode is disabled, stop shooter too
    if (!m_shooter.isTestModeEnabled()) {
      m_shooter.stopShooter();
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Runs while button is held
  }
}

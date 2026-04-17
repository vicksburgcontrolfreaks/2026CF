package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Staggered shooter motor startup to reduce current draw spikes.
 *
 * Sequence:
 * - T=0.0s: Start left motor (40A limit)
 * - T=0.3s: Start right motor (40A limit)
 * - T=0.6s: Start middle motor (40A limit)
 * - T=0.9s: All motors running, switch to idle current (30A)
 *
 * Total startup time: ~0.9s
 */
public class StaggeredShooterStartupCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final Timer m_timer;

  private static final double LEFT_START_TIME = 0.0;
  private static final double RIGHT_START_TIME = 0.3;
  private static final double MIDDLE_START_TIME = 0.6;
  private static final double COMPLETE_TIME = 0.9;

  private boolean m_leftStarted = false;
  private boolean m_rightStarted = false;
  private boolean m_middleStarted = false;

  public StaggeredShooterStartupCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
    m_timer = new Timer();
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_leftStarted = false;
    m_rightStarted = false;
    m_middleStarted = false;

    // Initialize startup state and set startup current limit
    m_shooter.beginStaggeredStartup();

    System.out.println("Staggered shooter startup initiated");
  }

  @Override
  public void execute() {
    double elapsed = m_timer.get();

    // Start motors in sequence with 0.3s gaps
    if (!m_leftStarted && elapsed >= LEFT_START_TIME) {
      m_shooter.startLeftShooter();
      m_leftStarted = true;
      System.out.println("Left shooter motor started (T=" + elapsed + "s)");
    }

    if (!m_rightStarted && elapsed >= RIGHT_START_TIME) {
      m_shooter.startRightShooter();
      m_rightStarted = true;
      System.out.println("Right shooter motor started (T=" + elapsed + "s)");
    }

    if (!m_middleStarted && elapsed >= MIDDLE_START_TIME) {
      m_shooter.startMiddleShooter();
      m_middleStarted = true;
      System.out.println("Middle shooter motor started (T=" + elapsed + "s)");
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();

    if (!interrupted) {
      // Complete startup - reduce to idle current limit
      m_shooter.completeStartup();
      System.out.println("Staggered startup complete - shooter ready");
    } else {
      System.out.println("Staggered startup interrupted");
    }
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() >= COMPLETE_TIME;
  }
}

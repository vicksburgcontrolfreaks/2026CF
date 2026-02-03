package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;

public class RunCollector extends Command {
  private final CollectorSubsystem m_collector;
  private final double m_speed;

  /**
   * Creates a new RunCollector command.
   *
   * @param collector The collector subsystem to use
   * @param speed The speed to run the collector motors (-1.0 to 1.0)
   */
  public RunCollector(CollectorSubsystem collector, double speed) {
    m_collector = collector;
    m_speed = speed;
    addRequirements(collector);
  }

  @Override
  public void initialize() {
    // Called when the command is initially scheduled
  }

  @Override
  public void execute() {
    // Called every scheduler run while the command is scheduled
    m_collector.run(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    // Called once the command ends or is interrupted
    m_collector.stop();
  }

  @Override
  public boolean isFinished() {
    // This command will run until interrupted (typically when button is released)
    return false;
  }
}

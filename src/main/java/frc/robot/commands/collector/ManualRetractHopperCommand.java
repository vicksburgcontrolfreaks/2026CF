package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;

public class ManualRetractHopperCommand extends Command {
    private final CollectorSubsystem m_collector;

    public ManualRetractHopperCommand(CollectorSubsystem collector) {
        m_collector = collector;
        addRequirements(collector);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_collector.setHopperManualSpeed(CollectorConstants.kHopperSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_collector.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return m_collector.isLimitSwitchPressed() || m_collector.getHopperPosition() <= 0.0;
    }
}

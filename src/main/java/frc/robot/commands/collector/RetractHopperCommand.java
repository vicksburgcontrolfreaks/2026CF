package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;

public class RetractHopperCommand extends Command {
    private final CollectorSubsystem m_hopper;

    public RetractHopperCommand(CollectorSubsystem hopper) {
        m_hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        if (m_hopper.getHopperPosition() != CollectorConstants.kHopperRetractedPosition) {
            m_hopper.setHopperPosition(CollectorConstants.kHopperRetractedPosition);
        }
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
}

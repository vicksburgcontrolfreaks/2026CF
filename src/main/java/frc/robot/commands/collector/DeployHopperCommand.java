package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;

public class DeployHopperCommand extends Command {
    private final CollectorSubsystem m_hopper;

    public DeployHopperCommand(CollectorSubsystem hopper) {
        m_hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        m_hopper.setHopperPosition(CollectorConstants.kHopperExtendedPosition);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return m_hopper.isHopperAtPosition(CollectorConstants.kHopperExtendedPosition);
    }
}

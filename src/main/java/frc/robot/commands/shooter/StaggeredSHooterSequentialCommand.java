package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StaggeredSHooterSequentialCommand extends SequentialCommandGroup {
    public StaggeredSHooterSequentialCommand(ShooterSubsystem shooter, DriveSubsystem drive, CollectorSubsystem collector, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        super(
            new StaggeredShooterStartupCommand(shooter),
            new AutoAimShootCommand(shooter, drive, collector, xSpeed, ySpeed)
        );
    }
}

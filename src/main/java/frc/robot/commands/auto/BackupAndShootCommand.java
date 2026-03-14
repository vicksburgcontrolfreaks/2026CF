package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * Simple autonomous command that backs up 1 meter and shoots.
 * This is a basic autonomous option for quick scoring.
 */
public class BackupAndShootCommand extends SequentialCommandGroup {

    private static final double BACKUP_SPEED = -0.5; // m/s (negative = backward)
    private static final double BACKUP_DISTANCE = 1.0; // meters
    private static final double BACKUP_TIME = BACKUP_DISTANCE / Math.abs(BACKUP_SPEED); // 2 seconds
    private static final double SHOOTER_SPINUP_TIME = 3.0; // seconds to spin up shooter
    private static final double SHOOT_TIME = 5.0; // seconds to run indexer/shoot
    private static final double MINIMUM_RPM = 2900.0; // Minimum shooter RPM (safety floor)

    public BackupAndShootCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            // Step 1: Start spinning up the shooter with minimum RPM enforcement
            new RunCommand(
                () -> activateShooterWithMinimum(shooterSubsystem),
                shooterSubsystem
            ).withTimeout(SHOOTER_SPINUP_TIME),

            // Step 2: Drive backward 1 meter while keeping shooter running
            new RunCommand(
                () -> {
                    // Drive backward at constant speed (robot-relative)
                    driveSubsystem.setChassisSpeeds(
                        new ChassisSpeeds(BACKUP_SPEED, 0, 0)
                    );
                    // Keep shooter running with minimum RPM
                    activateShooterWithMinimum(shooterSubsystem);
                },
                driveSubsystem,
                shooterSubsystem
            ).withTimeout(BACKUP_TIME),

            // Step 3: Stop driving
            new RunCommand(
                () -> driveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)),
                driveSubsystem
            ).withTimeout(0.1),

            // Step 4: Shoot (run floor motor and indexer)
            new RunCommand(
                () -> {
                    activateShooterWithMinimum(shooterSubsystem);
                    shooterSubsystem.runFloor(false); // Run floor motor forward
                    shooterSubsystem.runIndexer(false, false); // Run indexer forward
                },
                shooterSubsystem
            ).withTimeout(SHOOT_TIME),

            // Step 5: Stop all motors
            new RunCommand(
                () -> {
                    shooterSubsystem.stopShooter();
                    shooterSubsystem.StopFloor();
                    shooterSubsystem.StopIndexer();
                },
                shooterSubsystem
            ).withTimeout(0.1)
        );

        addRequirements(driveSubsystem, shooterSubsystem);
    }

    /**
     * Activates the shooter using vision-based RPM calculation, but enforces a minimum RPM.
     * This prevents shots from being too weak at close distances.
     */
    private void activateShooterWithMinimum(ShooterSubsystem shooterSubsystem) {
        // First, let the shooter calculate its target RPM based on vision distance
        shooterSubsystem.activateShooter();

        // Get the calculated RPM
        double calculatedRPM = shooterSubsystem.getMotorTargetRPM();

        // If it's below our minimum, override with the minimum RPM
        if (calculatedRPM < MINIMUM_RPM) {
            shooterSubsystem.setShooterRPM(MINIMUM_RPM);
        }
    }
}

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Simple autonomous command that drives forward 4 feet and shoots.
 * Does NOT rely on vision/AprilTags at all.
 * Shooter RPM is tunable via SmartDashboard.
 */
public class SimpleDriveAndShootCommand extends Command {
    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shooter;

    // Constants
    private static final double TARGET_DISTANCE_METERS = 1.2192; // 4 feet in meters
    private static final double DISTANCE_TOLERANCE = 0.05; // 5cm tolerance
    private static final double DRIVE_SPEED = 0.3; // 30% max speed for smooth motion
    private static final double SHOOT_DURATION_SECONDS = 3.0;
    private static final double DEFAULT_SHOOTER_RPM = 3000.0;

    // State tracking
    private double m_startX;
    private double m_startY;
    private boolean m_drivingComplete = false;
    private boolean m_shootingStarted = false;
    private double m_shootStartTime = 0;
    private double m_shooterRPM;

    public SimpleDriveAndShootCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
        m_drive = drive;
        m_shooter = shooter;

        addRequirements(drive, shooter);

        // Put default RPM on SmartDashboard for tuning
        SmartDashboard.putNumber("Simple Auto Shooter RPM", DEFAULT_SHOOTER_RPM);
    }

    @Override
    public void initialize() {
        // Record starting position
        m_startX = m_drive.getPose().getX();
        m_startY = m_drive.getPose().getY();

        // Reset state
        m_drivingComplete = false;
        m_shootingStarted = false;
        m_shootStartTime = 0;

        // Get tunable shooter RPM from SmartDashboard
        m_shooterRPM = SmartDashboard.getNumber("Simple Auto Shooter RPM", DEFAULT_SHOOTER_RPM);

        // Start spooling up shooter immediately
        m_shooter.activateShooterWithRPM(m_shooterRPM);
        m_shooter.enableRPMCap(); // Use capped RPM during spool-up

        System.out.println("SimpleDriveAndShootCommand started - Target: " + TARGET_DISTANCE_METERS + "m, RPM: " + m_shooterRPM);
    }

    @Override
    public void execute() {
        if (!m_drivingComplete) {
            // Phase 1: Drive forward
            executeDrivePhase();
        } else {
            // Phase 2: Shoot
            executeShootPhase();
        }
    }

    private void executeDrivePhase() {
        // Calculate distance traveled from start
        double currentX = m_drive.getPose().getX();
        double currentY = m_drive.getPose().getY();

        double deltaX = currentX - m_startX;
        double deltaY = currentY - m_startY;
        double distanceTraveled = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        SmartDashboard.putNumber("Simple Auto Distance", distanceTraveled);

        if (distanceTraveled >= TARGET_DISTANCE_METERS - DISTANCE_TOLERANCE) {
            // Target reached - stop driving
            m_drive.drive(0, 0, 0, false);
            m_drivingComplete = true;
            System.out.println("Drive complete - Distance traveled: " + distanceTraveled + "m");
        } else {
            // Keep driving forward at constant speed
            // X is forward/backward in robot coordinates (field-relative = false)
            double speedMPS = DRIVE_SPEED * frc.robot.constants.DriveConstants.kMaxSpeedMetersPerSecond;
            m_drive.drive(speedMPS, 0, 0, false); // Robot-relative forward
        }
    }

    private void executeShootPhase() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (!m_shootingStarted) {
            // First time in shoot phase - enable full RPM and start feeding when ready
            m_shooter.enableFullRPM();

            if (m_shooter.isReadyToFeed()) {
                // Shooter at target velocity - start feeding
                m_shooter.runIndexer(false);
                m_shootStartTime = currentTime;
                m_shootingStarted = true;
                System.out.println("Shooting started at RPM: " + m_shooterRPM);
            } else {
                // Still waiting for shooter to spin up
                SmartDashboard.putBoolean("Simple Auto Waiting for Shooter", true);
            }
        } else {
            // Continue shooting - indexer is already running
            SmartDashboard.putBoolean("Simple Auto Waiting for Shooter", false);
            SmartDashboard.putNumber("Simple Auto Shoot Time", currentTime - m_shootStartTime);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        m_drive.drive(0, 0, 0, false);

        // Stop shooter and indexer
        m_shooter.StopIndexer();
        m_shooter.stopShooter();
        m_shooter.enableRPMCap(); // Reset to capped mode

        if (interrupted) {
            System.out.println("SimpleDriveAndShootCommand interrupted");
        } else {
            System.out.println("SimpleDriveAndShootCommand completed");
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes after shooting for the specified duration
        if (m_shootingStarted) {
            double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            return (currentTime - m_shootStartTime) >= SHOOT_DURATION_SECONDS;
        }
        return false;
    }
}

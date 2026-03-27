package frc.robot.commands.test;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.ShooterTestLogger;
import frc.robot.utils.ShooterTestLogger.TestSequenceData;

/**
 * Autonomous command for shooter characterization testing.
 *
 * Test sequence:
 * 1. Activate shooter and measure spin-up time
 * 2. Wait for target velocity
 * 3. Run indexer + floor motor for configured duration
 * 4. Sample telemetry at 50 Hz during sequence
 * 5. Stop all motors
 * 6. Wait for operator to input scored count
 * 7. Log data to CSV
 *
 * Configure test parameters via NetworkTables before enabling:
 * - ShooterTest/Balls Loaded
 * - ShooterTest/Test Duration
 */
public class ShooterTestCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final ShooterTestLogger m_logger;

  private final Timer m_timer = new Timer();
  private final Timer m_samplingTimer = new Timer();

  private enum TestState {
    SPIN_UP,
    SHOOTING,
    COMPLETE,
    WAITING_FOR_INPUT
  }

  private TestState m_state;
  private double m_spinUpStartTime;
  private double m_spinUpTime;
  private double m_odometryDistance;
  private double m_measuredDistance;

  private int m_ballsLoaded;
  private double m_testDuration;
  private double m_shooterRPM;
  private double m_indexerRPM;
  private boolean m_dataLogged;

  private static final double VELOCITY_TOLERANCE = 100.0; // RPM
  private static final double SAMPLING_PERIOD = 0.02; // 50 Hz

  public ShooterTestCommand(ShooterSubsystem shooterSubsystem, ShooterTestLogger logger,
                            int ballsLoaded, double testDuration, double shooterRPM, double indexerRPM,
                            double measuredDistance) {
    m_shooterSubsystem = shooterSubsystem;
    m_logger = logger;
    m_ballsLoaded = ballsLoaded;
    m_testDuration = testDuration;
    m_shooterRPM = shooterRPM;
    m_indexerRPM = indexerRPM;
    m_measuredDistance = measuredDistance;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    m_state = TestState.SPIN_UP;
    m_dataLogged = false;

    // Record initial conditions (odometry distance)
    m_odometryDistance = m_shooterSubsystem.getDistanceToSpeaker();

    // Start logger
    m_logger.startSequence();
    m_logger.setStatus("Test " + m_logger.getSequenceNumber() + ": Spinning up shooter");

    // Start shooter with configured RPM
    m_shooterSubsystem.activateShooterWithRPM(m_shooterRPM);
    m_spinUpStartTime = Timer.getFPGATimestamp();

    m_timer.restart();
    m_samplingTimer.restart();
  }

  @Override
  public void execute() {
    // Sample telemetry at 50 Hz
    if (m_samplingTimer.hasElapsed(SAMPLING_PERIOD)) {
      m_samplingTimer.restart();

      double shooterRPM = m_shooterSubsystem.getAverageShooterRPM();
      double indexerRPM = m_shooterSubsystem.getAverageIndexerRPM();
      double shooterCurrent = m_shooterSubsystem.getAverageShooterCurrent();
      double indexerCurrent = m_shooterSubsystem.getAverageIndexerCurrent();
      double voltage = RobotController.getBatteryVoltage();

      m_logger.addSample(shooterRPM, indexerRPM, shooterCurrent, indexerCurrent, voltage);
    }

    switch (m_state) {
      case SPIN_UP:
        // Wait for shooter to reach target velocity
        if (m_shooterSubsystem.isAtTargetVelocity(VELOCITY_TOLERANCE)) {
          m_spinUpTime = Timer.getFPGATimestamp() - m_spinUpStartTime;
          m_logger.setStatus("Test " + m_logger.getSequenceNumber() + ": Shooting");

          // Start feeding with configured RPM
          m_shooterSubsystem.runIndexerWithRPM(m_indexerRPM, false);
          m_shooterSubsystem.runFloor(false);

          m_state = TestState.SHOOTING;
          m_timer.restart();
        }
        break;

      case SHOOTING:
        // Run indexer for configured duration
        if (m_timer.hasElapsed(m_testDuration)) {
          // Stop all motors
          m_shooterSubsystem.StopIndexer();
          m_shooterSubsystem.StopFloor();
          m_shooterSubsystem.stopShooter();

          m_logger.setStatus("Test " + m_logger.getSequenceNumber() + ": Complete - Enter scored count");
          m_state = TestState.COMPLETE;
          m_timer.restart();
        }
        break;

      case COMPLETE:
        // Wait 1 second before transitioning to input wait state
        if (m_timer.hasElapsed(1.0)) {
          m_state = TestState.WAITING_FOR_INPUT;
        }
        break;

      case WAITING_FOR_INPUT:
        // Command ends when operator inputs scored count via controller
        // (handled by external button binding calling logAndFinish())
        break;
    }
  }

  /**
   * Call this method from controller binding after operator inputs scored count
   * @param ballsScored Number of balls that scored
   */
  public void logAndFinish(int ballsScored) {
    if (!m_dataLogged && m_state == TestState.WAITING_FOR_INPUT) {
      // Create test data record
      TestSequenceData data = new TestSequenceData();
      data.odometryDistance = m_odometryDistance;
      data.measuredDistance = m_measuredDistance;
      data.ballsLoaded = m_ballsLoaded;
      data.ballsScored = ballsScored;
      data.targetRPM = m_shooterRPM;
      data.indexerTargetRPM = m_indexerRPM;
      data.spinUpTime = m_spinUpTime;
      data.sequenceDuration = m_testDuration;
      data.notes = "";

      // Log to CSV
      m_logger.logSequence(data);
      m_dataLogged = true;

      m_logger.setStatus("Test " + (m_logger.getSequenceNumber() - 1) + ": Logged");
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Safety: stop all motors
    m_shooterSubsystem.StopIndexer();
    m_shooterSubsystem.StopFloor();
    m_shooterSubsystem.stopShooter();

    if (interrupted) {
      m_logger.setStatus("Test interrupted");
    }
  }

  @Override
  public boolean isFinished() {
    // Command finishes after data is logged
    return m_dataLogged;
  }
}

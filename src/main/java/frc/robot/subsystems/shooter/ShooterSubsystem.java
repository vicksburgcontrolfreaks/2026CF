package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Shooter subsystem that controls two motors for shooting game pieces.
 */
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_topMotor;
  private final SparkMax m_bottomMotor;

  /** Creates a new Shooter subsystem. */
  public ShooterSubsystem() {
    // Initialize motors with CAN IDs from configuration
    m_topMotor = new SparkMax(ShooterConstants.kTopMotorId, MotorType.kBrushless);
    m_bottomMotor = new SparkMax(ShooterConstants.kBottomMotorId, MotorType.kBrushless);

    // Configure motors with settings from configuration file
    m_topMotor.configure(ShooterConstants.topMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_bottomMotor.configure(ShooterConstants.bottomMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Runs both shooter motors at the specified speed.
   *
   * @param speed The speed to run the motors (-1.0 to 1.0)
   */
  public void run(double speed) {
    m_topMotor.set(speed);
    m_bottomMotor.set(speed);
  }

  /**
   * Runs the shooter motors at different speeds.
   *
   * @param topSpeed The speed for the top motor (-1.0 to 1.0)
   * @param bottomSpeed The speed for the bottom motor (-1.0 to 1.0)
   */
  public void run(double topSpeed, double bottomSpeed) {
    m_topMotor.set(topSpeed);
    m_bottomMotor.set(bottomSpeed);
  }

  /**
   * Runs both shooter motors at the default speed.
   */
  public void runDefault() {
    run(ShooterConstants.kDefaultShooterSpeed);
  }

  /**
   * Runs both shooter motors at slow speed.
   */
  public void runSlow() {
    run(ShooterConstants.kSlowShooterSpeed);
  }

  /**
   * Stops both shooter motors.
   */
  public void stop() {
    m_topMotor.set(0);
    m_bottomMotor.set(0);
  }

  /**
   * Gets the current speed of the top motor.
   *
   * @return The speed of the top motor
   */
  public double getTopMotorSpeed() {
    return m_topMotor.getAppliedOutput();
  }

  /**
   * Gets the current speed of the bottom motor.
   *
   * @return The speed of the bottom motor
   */
  public double getBottomMotorSpeed() {
    return m_bottomMotor.getAppliedOutput();
  }

  /**
   * Gets whether the shooter is currently running.
   *
   * @return True if either motor is running, false otherwise
   */
  public boolean isRunning() {
    return Math.abs(m_topMotor.getAppliedOutput()) > 0.01 ||
           Math.abs(m_bottomMotor.getAppliedOutput()) > 0.01;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

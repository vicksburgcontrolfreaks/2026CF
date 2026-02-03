package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climber subsystem that controls a telescoping climber with a winch system.
 * Uses a single motor with a 60:1 gear ratio.
 */
public class Climber extends SubsystemBase {
  private final SparkMax m_climberMotor;

  /** Creates a new Climber subsystem. */
  public Climber() {
    // Initialize motor with CAN ID from configuration
    m_climberMotor = new SparkMax(ClimberConfiguration.kClimberMotorId, MotorType.kBrushless);

    // Configure motor with settings from configuration file
    m_climberMotor.configure(ClimberConfiguration.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Extends the climber at the configured speed.
   */
  public void extend() {
    m_climberMotor.set(ClimberConfiguration.kExtendSpeed);
  }

  /**
   * Retracts the climber at the configured speed.
   */
  public void retract() {
    m_climberMotor.set(ClimberConfiguration.kRetractSpeed);
  }

  /**
   * Extends the climber slowly for fine adjustment.
   */
  public void extendSlow() {
    m_climberMotor.set(ClimberConfiguration.kSlowSpeed);
  }

  /**
   * Retracts the climber slowly for fine adjustment.
   */
  public void retractSlow() {
    m_climberMotor.set(-ClimberConfiguration.kSlowSpeed);
  }

  /**
   * Runs the climber at a custom speed.
   *
   * @param speed The speed to run the motor (-1.0 to 1.0, positive = extend)
   */
  public void run(double speed) {
    m_climberMotor.set(speed);
  }

  /**
   * Stops the climber motor.
   */
  public void stop() {
    m_climberMotor.set(0);
  }

  /**
   * Gets the current speed of the climber motor.
   *
   * @return The current motor speed
   */
  public double getSpeed() {
    return m_climberMotor.getAppliedOutput();
  }

  /**
   * Gets whether the climber is currently running.
   *
   * @return True if the motor is running, false otherwise
   */
  public boolean isRunning() {
    return Math.abs(m_climberMotor.getAppliedOutput()) > 0.01;
  }

  /**
   * Gets the current motor current draw.
   *
   * @return The current in amps
   */
  public double getCurrent() {
    return m_climberMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Can be used for monitoring current draw or position limits if encoders are added
  }
}

package frc.robot.subsystems.collector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
  private final SparkMax m_leftMotor;
  private final SparkMax m_rightMotor;

  public Collector() {
    // Initialize motors
    m_leftMotor = new SparkMax(CollectorConfig.kLeftMotorId, MotorType.kBrushless);
    m_rightMotor = new SparkMax(CollectorConfig.kRightMotorId, MotorType.kBrushless);

    // Configure motors
    m_leftMotor.configure(CollectorConfig.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rightMotor.configure(CollectorConfig.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Runs both collector motors at the specified speed.
   *
   * @param speed The speed to run the motors (-1.0 to 1.0)
   */
  public void run(double speed) {
    m_leftMotor.set(speed);
    m_rightMotor.set(speed);
  }

  /**
   * Runs the collector motors at different speeds.
   *
   * @param leftSpeed The speed for the left motor (-1.0 to 1.0)
   * @param rightSpeed The speed for the right motor (-1.0 to 1.0)
   */
  public void run(double leftSpeed, double rightSpeed) {
    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
  }

  /**
   * Stops both collector motors.
   */
  public void stop() {
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

package frc.robot.subsystems.collector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Collector subsystem with motorized deployment control.
 * The collector moves along a rail using deployment motors.
 * When deployed, the collector extends and runs the collection motors.
 * When retracted, the collector retracts and motors are turned off.
 */
public class Collector extends SubsystemBase {
  // Collection motors (intake rollers)
  private final SparkMax m_leftMotor;
  private final SparkMax m_rightMotor;

  // Deployment motors (move collector on rail)
  private final SparkMax m_leftDeploymentMotor;
  private final SparkMax m_rightDeploymentMotor;

  //
  // Encoder for tracking deployment position
  private final RelativeEncoder m_deploymentEncoder;

  // State tracking
  private boolean m_isDeployed;
  private double m_targetPosition;

  public Collector() {
    // Initialize collection motors
    m_leftMotor = new SparkMax(CollectorConfig.kLeftMotorId, MotorType.kBrushless);
    m_rightMotor = new SparkMax(CollectorConfig.kRightMotorId, MotorType.kBrushless);

    // Initialize deployment motors
    m_leftDeploymentMotor = new SparkMax(CollectorConfig.kLeftDeploymentMotorId, MotorType.kBrushless);
    m_rightDeploymentMotor = new SparkMax(CollectorConfig.kRightDeploymentMotorId, MotorType.kBrushless);

    // Configure collection motors
    m_leftMotor.configure(CollectorConfig.collectionConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rightMotor.configure(CollectorConfig.collectionConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Configure deployment motors
    m_leftDeploymentMotor.configure(CollectorConfig.deploymentConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rightDeploymentMotor.configure(CollectorConfig.deploymentConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Get encoder from left deployment motor for position tracking
    m_deploymentEncoder = m_leftDeploymentMotor.getEncoder();

    // Reset encoder position (assumes starting in retracted position)
    m_deploymentEncoder.setPosition(CollectorConfig.kRetractedPosition);

    // Start in retracted position
    m_isDeployed = false;
  }

  /**
   * Deploys the collector and starts running the motors.
   * Moves the collector along the rail to the deployed position and activates collection.
   */
  public void deploy() {
    m_targetPosition = CollectorConfig.kDeployedPosition;
    m_isDeployed = true;
  }

  /**
   * Retracts the collector and stops the motors.
   * Moves the collector along the rail to the retracted position and turns off motors.
   */
  public void retract() {
    m_targetPosition = CollectorConfig.kRetractedPosition;
    m_isDeployed = false;
    // Stop collection motors when retracting
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }

  /**
   * Toggles the collector between deployed and retracted states.
   */
  public void toggle() {
    if (m_isDeployed) {
      retract();
    } else {
      deploy();
    }
  }

  /**
   * Runs both collector motors at the specified speed.
   * Note: Motors will only run effectively when collector is deployed.
   *
   * @param speed The speed to run the motors (-1.0 to 1.0)
   */
  public void run(double speed) {
    // Only allow running motors if deployed
    if (m_isDeployed) {
      m_leftMotor.set(speed);
      m_rightMotor.set(speed);
    }
  }

  /**
   * Runs the collector motors at different speeds.
   * Note: Motors will only run effectively when collector is deployed.
   *
   * @param leftSpeed The speed for the left motor (-1.0 to 1.0)
   * @param rightSpeed The speed for the right motor (-1.0 to 1.0)
   */
  public void run(double leftSpeed, double rightSpeed) {
    // Only allow running motors if deployed
    if (m_isDeployed) {
      m_leftMotor.set(leftSpeed);
      m_rightMotor.set(rightSpeed);
    }
  }

  /**
   * Stops both collector motors.
   */
  public void stop() {
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }

  /**
   * Checks if the collector is currently deployed.
   * @return true if deployed, false if retracted
   */
  public boolean isDeployed() {
    return m_isDeployed;
  }

  /**
   * Checks if the collector has reached its target position.
   * @return true if at target position within tolerance
   */
  public boolean atTargetPosition() {
    double currentPosition = m_deploymentEncoder.getPosition();
    return Math.abs(currentPosition - m_targetPosition) < CollectorConfig.kPositionTolerance;
  }

  /**
   * Gets the current position of the collector.
   * @return Position in encoder rotations
   */
  public double getPosition() {
    return m_deploymentEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // Control deployment motors to reach target position
    double currentPosition = m_deploymentEncoder.getPosition();
    double error = m_targetPosition - currentPosition;

    if (Math.abs(error) > CollectorConfig.kPositionTolerance) {
      // Move toward target position
      double speed = Math.signum(error) * CollectorConfig.kDeploymentSpeed;
      m_leftDeploymentMotor.set(speed);
      m_rightDeploymentMotor.set(speed);
    } else {
      // At target position, stop deployment motors
      m_leftDeploymentMotor.set(0);
      m_rightDeploymentMotor.set(0);

      // If deployed and at position, start collection motors
      if (m_isDeployed && atTargetPosition()) {
        m_leftMotor.set(CollectorConfig.kCollectionSpeed);
        m_rightMotor.set(CollectorConfig.kCollectionSpeed);
      }
    }
  }
}

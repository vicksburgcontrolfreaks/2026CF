package frc.robot.subsystems.collector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

/**
 * Collector subsystem with motorized deployment control.
 * The collector moves along a rail using deployment motors.
 * When deployed, the collector extends and runs the collection motors.
 * When retracted, the collector retracts and motors are turned off.
 */
public class CollectorSubsystem extends SubsystemBase {
  // Collection motors (intake rollers)
  private final SparkMax m_leftMotor;
  private final SparkMax m_rightMotor;

  // Deployment motors (move collector on rail)
  private final SparkMax m_leftDeploymentMotor;
  private final SparkMax m_rightDeploymentMotor;

  private final RelativeEncoder m_deploymentEncoder;

  // State tracking
  private boolean m_isDeployed;
  private double m_targetPosition;

  public CollectorSubsystem() {
    m_leftMotor = new SparkMax(CollectorConstants.kLeftMotorId, MotorType.kBrushless);
    m_rightMotor = new SparkMax(CollectorConstants.kRightMotorId, MotorType.kBrushless);

    m_leftDeploymentMotor = new SparkMax(CollectorConstants.kLeftDeploymentMotorId, MotorType.kBrushless);
    m_rightDeploymentMotor = new SparkMax(CollectorConstants.kRightDeploymentMotorId, MotorType.kBrushless);

    m_leftMotor.configure(CollectorConstants.collectionConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rightMotor.configure(CollectorConstants.collectionConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_leftDeploymentMotor.configure(CollectorConstants.deploymentConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rightDeploymentMotor.configure(CollectorConstants.deploymentConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_deploymentEncoder = m_leftDeploymentMotor.getEncoder();

    m_deploymentEncoder.setPosition(CollectorConstants.kRetractedPosition);

    m_isDeployed = false;
  }

  public void deploy() {
    m_targetPosition = CollectorConstants.kDeployedPosition;
    m_isDeployed = true;
  }

  public void retract() {
    m_targetPosition = CollectorConstants.kRetractedPosition;
    m_isDeployed = false;
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }

  public void toggle() {
    if (m_isDeployed) {
      retract();
    } else {
      deploy();
    }
  }

  public void run(double speed) {
    if (m_isDeployed) {
      m_leftMotor.set(speed);
      m_rightMotor.set(speed);
    }
  }

  public void run(double leftSpeed, double rightSpeed) {
    if (m_isDeployed) {
      m_leftMotor.set(leftSpeed);
      m_rightMotor.set(rightSpeed);
    }
  }

  public void stop() {
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }

  public boolean isDeployed() {
    return m_isDeployed;
  }

  public boolean atTargetPosition() {
    double currentPosition = m_deploymentEncoder.getPosition();
    return Math.abs(currentPosition - m_targetPosition) < CollectorConstants.kPositionTolerance;
  }

  public double getPosition() {
    return m_deploymentEncoder.getPosition();
  }

  @Override
  public void periodic() {
    double currentPosition = m_deploymentEncoder.getPosition();
    double error = m_targetPosition - currentPosition;

    if (Math.abs(error) > CollectorConstants.kPositionTolerance) {
      double speed = Math.signum(error) * CollectorConstants.kDeploymentSpeed;
      m_leftDeploymentMotor.set(speed);
      m_rightDeploymentMotor.set(speed);
    } else {
      m_leftDeploymentMotor.set(0);
      m_rightDeploymentMotor.set(0);

      if (m_isDeployed && atTargetPosition()) {
        m_leftMotor.set(CollectorConstants.kCollectionSpeed);
        m_rightMotor.set(CollectorConstants.kCollectionSpeed);
      }
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.TelemetryConstants;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex m_rightShooterMotor;
  private final SparkFlex m_floorMotor;
  private final SparkFlex m_indexerMotor;
  private final SparkFlex m_leftShooterMotor;

  private int m_telemetryCounter = 0;

  private final NetworkTable m_telemetryTable;
  private final DoublePublisher m_rightShooterSpeedPub;
  private final DoublePublisher m_floorMotorSpeedPub;
  private final DoublePublisher m_indexerSpeedPub;
  private final DoublePublisher m_leftShooterSpeedPub;
  private final DoublePublisher m_rightShooterCurrentPub;
  private final DoublePublisher m_floorMotorCurrentPub;
  private final DoublePublisher m_indexerCurrentPub;
  private final DoublePublisher m_leftShooterCurrentPub;
  private final DoublePublisher m_rightShooterVelocityPub;
  private final DoublePublisher m_floorMotorVelocityPub;
  private final DoublePublisher m_indexerVelocityPub;
  private final DoublePublisher m_leftShooterVelocityPub;
  private final DoublePublisher m_targetRPMPub;
  private final DoublePublisher m_distanceToTargetPub;
  private final StringPublisher m_debugMessagePub;

  private double m_currentTargetRPM = ShooterConstants.kShooterTargetRPM;
  private double m_lastDistanceToTarget = 0.0;

  //private static double kTargetRPM = 3000; // 40% of max velocity
  // max rpm 6784 

  public ShooterSubsystem() {
    m_rightShooterMotor = new SparkFlex(ShooterConstants.kRightShooterId, MotorType.kBrushless);
    m_floorMotor = new SparkFlex(ShooterConstants.kFloorMotorId, MotorType.kBrushless);
    m_indexerMotor = new SparkFlex(ShooterConstants.kIndexerMotorId, MotorType.kBrushless);
    m_leftShooterMotor = new SparkFlex(ShooterConstants.kLeftShooterId, MotorType.kBrushless);

    m_rightShooterMotor.configure(ShooterConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_floorMotor.configure(ShooterConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_indexerMotor.configure(ShooterConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure left shooter to follow right shooter (inverted because they're on opposite sides of the axle)
    var leftConfig = new com.revrobotics.spark.config.SparkFlexConfig();
    leftConfig.follow(ShooterConstants.kRightShooterId, true);
    m_leftShooterMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_indexerMotor.set(0);
    m_floorMotor.set(0);

    m_telemetryTable = NetworkTableInstance.getDefault().getTable("Shooter");
    m_rightShooterSpeedPub   = m_telemetryTable.getDoubleTopic("Right Shooter Speed").publish();
    m_floorMotorSpeedPub     = m_telemetryTable.getDoubleTopic("Floor Motor Speed").publish();
    m_indexerSpeedPub        = m_telemetryTable.getDoubleTopic("Indexer Speed").publish();
    m_leftShooterSpeedPub    = m_telemetryTable.getDoubleTopic("Left Shooter Speed").publish();
    m_rightShooterCurrentPub = m_telemetryTable.getDoubleTopic("Right Shooter Current").publish();
    m_floorMotorCurrentPub   = m_telemetryTable.getDoubleTopic("Floor Motor Current").publish();
    m_indexerCurrentPub      = m_telemetryTable.getDoubleTopic("Indexer Current").publish();
    m_leftShooterCurrentPub  = m_telemetryTable.getDoubleTopic("Left Shooter Current").publish();
    m_rightShooterVelocityPub  = m_telemetryTable.getDoubleTopic("Right Shooter Velocity").publish();
    m_floorMotorVelocityPub    = m_telemetryTable.getDoubleTopic("Floor Motor Velocity").publish();
    m_indexerVelocityPub       = m_telemetryTable.getDoubleTopic("Indexer Velocity").publish();
    m_leftShooterVelocityPub   = m_telemetryTable.getDoubleTopic("Left Shooter Velocity").publish();
    m_targetRPMPub             = m_telemetryTable.getDoubleTopic("Target RPM").publish();
    m_distanceToTargetPub      = m_telemetryTable.getDoubleTopic("Distance to Target").publish();
    m_debugMessagePub          = m_telemetryTable.getStringTopic("Debug Message").publish();
  }

  public void runFloor(boolean reversed) {
    double targetRPM = ShooterConstants.kFloorMotorTargetRPM;
    if (reversed) {
      targetRPM = -targetRPM;
    }

    m_floorMotor.getClosedLoopController().setSetpoint(
      targetRPM,
      ControlType.kVelocity
    );
  }

  public void StopFloor() {
    m_floorMotor.set(0);
  }

  public void StopIndexer() {
    m_indexerMotor.set(0);
  }

  /**
   * Run the shooter with dynamic RPM based on distance to target from vision
   * @param vision PhotonVisionSubsystem to get distance from (pass null to use default RPM)
   */
  public void runShooter(PhotonVisionSubsystem vision) {
    double targetRPM;

    // DYNAMIC RPM CODE - Uses robot pose and speaker position to calculate distance
    if (vision != null) {
      double distance = vision.getDistanceToSpeaker();
      m_lastDistanceToTarget = distance;

      if (distance > 0) {
        targetRPM = ShooterConstants.getRPMForDistance(distance);
        m_debugMessagePub.set("Dynamic Shooter: Distance = " + String.format("%.2f", distance) +
                              "m, Target RPM = " + String.format("%.0f", targetRPM));
      } else {
        // Fall back to default RPM if distance calculation fails
        targetRPM = ShooterConstants.kShooterTargetRPM;
        m_debugMessagePub.set("Dynamic Shooter: Distance calculation failed, using default RPM = " + targetRPM);
      }
    } else {
      // Use default RPM if no vision subsystem provided
      targetRPM = ShooterConstants.kShooterTargetRPM;
      m_lastDistanceToTarget = 0.0;
    }

    m_currentTargetRPM = targetRPM;

    m_rightShooterMotor.getClosedLoopController().setSetpoint(
      targetRPM,
      ControlType.kVelocity
    );

    m_leftShooterMotor.getClosedLoopController().setSetpoint(
      -targetRPM,
      ControlType.kVelocity
    );
  }

  /**
   * Get the current target RPM
   * @return The current target RPM
   */
  public double getCurrentTargetRPM() {
    return m_currentTargetRPM;
  }

  /**
   * Check if the shooter is at the target velocity
   * @param tolerance Acceptable RPM tolerance (e.g., 100 RPM)
   * @return True if both shooter motors are within tolerance of target
   */
  public boolean isAtTargetVelocity(double tolerance) {
    double rightVelocity = Math.abs(m_rightShooterMotor.getEncoder().getVelocity());
    double leftVelocity = Math.abs(m_leftShooterMotor.getEncoder().getVelocity());

    return Math.abs(rightVelocity - m_currentTargetRPM) <= tolerance &&
           Math.abs(leftVelocity - m_currentTargetRPM) <= tolerance;
  }

  public void runIndexer(boolean reversed, boolean manual) {
    double rpm = ShooterConstants.kIndexerMotorTargetRPM;
    if (reversed) {
      rpm = -rpm;
    }

    if (manual) {
      m_indexerMotor.set(-0.75);
    } else {
      m_indexerMotor.getClosedLoopController().setSetpoint(
        rpm,
        ControlType.kVelocity
      );
    }
  }

  /**
   * Run the indexer slowly in reverse for collection
   */
  public void runIndexerSlowReverse() {
    m_indexerMotor.set(-0.2);
  }

  public void stopAll() {
    m_rightShooterMotor.set(0);
    m_floorMotor.set(0);
    m_indexerMotor.set(0);
    m_leftShooterMotor.set(0);
  }

  public boolean isAnyMotorOverCurrent(double threshold) {
    return m_rightShooterMotor.getOutputCurrent() > threshold ||
           m_floorMotor.getOutputCurrent() > threshold ||
           m_indexerMotor.getOutputCurrent() > threshold ||
           m_leftShooterMotor.getOutputCurrent() > threshold;
  }

  @Override
  public void periodic() {
    m_telemetryCounter++;
    if (m_telemetryCounter >= TelemetryConstants.kTelemetryUpdatePeriod) {
      m_telemetryCounter = 0;

      m_rightShooterSpeedPub.set(m_rightShooterMotor.get());
      m_floorMotorSpeedPub.set(m_floorMotor.get());
      m_indexerSpeedPub.set(m_indexerMotor.get());
      m_leftShooterSpeedPub.set(m_leftShooterMotor.get());

      m_rightShooterCurrentPub.set(m_rightShooterMotor.getOutputCurrent());
      m_floorMotorCurrentPub.set(m_floorMotor.getOutputCurrent());
      m_indexerCurrentPub.set(m_indexerMotor.getOutputCurrent());
      m_leftShooterCurrentPub.set(m_leftShooterMotor.getOutputCurrent());

      m_rightShooterVelocityPub.set(m_rightShooterMotor.getEncoder().getVelocity());
      m_floorMotorVelocityPub.set(m_floorMotor.getEncoder().getVelocity());
      m_indexerVelocityPub.set(m_indexerMotor.getEncoder().getVelocity());
      m_leftShooterVelocityPub.set(m_leftShooterMotor.getEncoder().getVelocity());
      m_targetRPMPub.set(m_currentTargetRPM);
      m_distanceToTargetPub.set(m_lastDistanceToTarget);
    }
  }

  public static double getKShooterTargetRPM() {
    return ShooterConstants.kShooterTargetRPM;
  }
}

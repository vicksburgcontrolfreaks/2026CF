// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;

public class ShooterAdjustments extends SubsystemBase {
  private final SparkMax m_shooterTop;
  private final SparkMax m_shooterFront;

  // Current speed of the front shooter (starts at initial speed)
  private double m_frontShooterSpeed;

  // Flag to track if shooter is running
  private boolean m_isRunning;

  /** Creates a new ShooterAdjustments subsystem. */
  public ShooterAdjustments() {
    // Initialize the shooter motors
    m_shooterTop = new SparkMax(SwerveConstants.kShooterTop, MotorType.kBrushless);
    m_shooterFront = new SparkMax(SwerveConstants.kShooterFront, MotorType.kBrushless);

    // Configure motors using the config object
    m_shooterTop.configure(Configs.Shooter.shooterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_shooterFront.configure(Configs.Shooter.shooterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Initialize state
    m_frontShooterSpeed = ShooterConstants.kFrontShooterStartSpeed;
    m_isRunning = false;
  }

  /**
   * Starts the shooter motors.
   * Top shooter runs immediately at 80% power.
   * Front shooter starts at 20% and will ramp up in periodic().
   */
  public void startShooter() {
    m_isRunning = true;
    // Reset front shooter speed to starting value
    m_frontShooterSpeed = ShooterConstants.kFrontShooterStartSpeed;
    // Top shooter runs at constant 80%
    m_shooterTop.set(ShooterConstants.kTopShooterSpeed);
    // Front shooter starts at 20%
    m_shooterFront.set(m_frontShooterSpeed);
  }

  /**
   * Stops both shooter motors and resets the front shooter speed.
   */
  public void stopShooter() {
    m_isRunning = false;
    m_shooterTop.set(0.0);
    m_shooterFront.set(0.0);
    // Reset front shooter speed for next run
    m_frontShooterSpeed = ShooterConstants.kFrontShooterStartSpeed;
  }

  /**
   * Gets whether the shooter is currently running.
   */
  public boolean isRunning() {
    return m_isRunning;
  }

  /**
   * Gets the current front shooter speed.
   */
  public double getFrontShooterSpeed() {
    return m_frontShooterSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (every 20ms)

    // If shooter is running, ramp up the front shooter speed
    if (m_isRunning) {
      // Check if we haven't reached the max speed yet
      if (m_frontShooterSpeed < ShooterConstants.kFrontShooterMaxSpeed) {
        // Increase front shooter speed by proportional value
        m_frontShooterSpeed += ShooterConstants.kPShooter;

        // Clamp to max speed (soft cap at 0.8)
        if (m_frontShooterSpeed >= ShooterConstants.kFrontShooterMaxSpeed) {
          m_frontShooterSpeed = ShooterConstants.kFrontShooterMaxSpeed;
        }

        // Set the new speed to the motor
        m_shooterFront.set(m_frontShooterSpeed);
      }
    }
  }
}

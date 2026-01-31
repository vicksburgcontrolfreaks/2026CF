// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climber subsystem that controls a motor to raise and lower the robot's climber mechanism.
 */
public class Climber extends SubsystemBase {
  private final SparkMax m_motor;

  /** Creates a new Climber subsystem. */
  public Climber() {
    // Initialize the climber motor
    m_motor = new SparkMax(ClimberConfig.kMotorId, MotorType.kBrushless);

    // Configure motor
    m_motor.configure(ClimberConfig.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Moves the climber up at the configured climb speed.
   */
  public void climbUp() {
    m_motor.set(ClimberConfig.kClimbUpSpeed);
  }

  /**
   * Moves the climber down at the configured climb speed.
   */
  public void climbDown() {
    m_motor.set(ClimberConfig.kClimbDownSpeed);
  }

  /**
   * Runs the climber motor at a specified speed.
   *
   * @param speed The speed to run the motor (-1.0 to 1.0)
   *              Positive = up, Negative = down
   */
  public void move(double speed) {
    m_motor.set(speed);
  }

  /**
   * Stops the climber motor.
   */
  public void stop() {
    m_motor.set(0.0);
  }

  /**
   * Gets the current motor speed.
   */
  public double getSpeed() {
    return m_motor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

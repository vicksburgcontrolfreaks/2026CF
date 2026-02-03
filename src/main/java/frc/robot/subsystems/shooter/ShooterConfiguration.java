package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Configuration file for the Shooter subsystem.
 * Contains motor CAN IDs, current limits, inversions, and motor configurations.
 */
public final class ShooterConfiguration {
  // CAN IDs for shooter motors - UPDATE THESE TO MATCH YOUR ROBOT
  public static final int kTopMotorId = 0;
  public static final int kBottomMotorId = 1;

  // Current limits
  public static final int kMotorCurrentLimit = 40; // Amps

  // Motor inversions
  public static final boolean kTopMotorInverted = false;
  public static final boolean kBottomMotorInverted = false;

  // Shooter speed settings
  public static final double kDefaultShooterSpeed = 0.8; // 80% power
  public static final double kSlowShooterSpeed = 0.5; // 50% power for close shots
  public static final double kMaxShooterSpeed = 1.0; // Maximum speed

  // Motor idle mode (brake or coast)
  public static final IdleMode kIdleMode = IdleMode.kBrake;

  // Motor configuration for top shooter motor
  public static final SparkMaxConfig topMotorConfig = new SparkMaxConfig();

  // Motor configuration for bottom shooter motor
  public static final SparkMaxConfig bottomMotorConfig = new SparkMaxConfig();

  static {
    // Configure top motor
    topMotorConfig
        .idleMode(kIdleMode)
        .smartCurrentLimit(kMotorCurrentLimit)
        .inverted(kTopMotorInverted);

    // Configure bottom motor
    bottomMotorConfig
        .idleMode(kIdleMode)
        .smartCurrentLimit(kMotorCurrentLimit)
        .inverted(kBottomMotorInverted);
  }
}

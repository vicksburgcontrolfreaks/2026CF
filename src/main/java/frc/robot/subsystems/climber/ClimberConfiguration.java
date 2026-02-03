package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Configuration file for the Climber subsystem.
 * Contains motor CAN ID, current limits, inversions, and motor configuration.
 *
 * This climber uses a 60:1 gear ratio winch system to extend/retract a telescoping climber.
 */
public final class ClimberConfiguration {
  // CAN ID for climber motor - UPDATE THIS TO MATCH YOUR ROBOT
  public static final int kClimberMotorId = 12;

  // Current limits
  public static final int kMotorCurrentLimit = 40; // Amps - adjust based on your winch requirements
  public static final int kMotorStallLimit = 60; // Amps - higher limit for stall conditions

  // Motor inversion
  public static final boolean kMotorInverted = false;

  // Climber speeds (positive = extend, negative = retract)
  public static final double kExtendSpeed = 0.8; // Speed when extending climber
  public static final double kRetractSpeed = -0.8; // Speed when retracting climber
  public static final double kSlowSpeed = 0.3; // Slow speed for fine adjustments

  // Motor idle mode (brake prevents climber from falling when motor stops)
  public static final IdleMode kIdleMode = IdleMode.kBrake;

  // Gear ratio information (for reference)
  public static final double kGearRatio = 60.0; // 60:1 reduction

  // Motor configuration
  public static final SparkMaxConfig config = new SparkMaxConfig();

  static {
    config
        .idleMode(kIdleMode)
        .smartCurrentLimit(kMotorCurrentLimit)
        .inverted(kMotorInverted);
  }
}

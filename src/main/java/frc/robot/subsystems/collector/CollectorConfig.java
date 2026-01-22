package frc.robot.subsystems.collector;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class CollectorConfig {
  // CAN IDs for collector motors - UPDATE THESE TO MATCH YOUR ROBOT
  public static final int kLeftMotorId = 10;
  public static final int kRightMotorId = 11;

  // Current limits
  public static final int kMotorCurrentLimit = 40; // Amps

  // Motor inversions
  public static final boolean kLeftMotorInverted = false;
  public static final boolean kRightMotorInverted = true; // Typically opposite side motors are inverted

  // Motor configuration
  public static final SparkMaxConfig config = new SparkMaxConfig();

  static {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kMotorCurrentLimit)
        .inverted(kLeftMotorInverted);
  }
}

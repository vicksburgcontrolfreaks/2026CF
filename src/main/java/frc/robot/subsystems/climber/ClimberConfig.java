package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class ClimberConfig {
  // CAN ID for climber motor
  public static final int kMotorId = 12;

  // Current limits
  public static final int kMotorCurrentLimit = 40; // Amps

  // Motor inversion
  public static final boolean kMotorInverted = false;

  // Climber speeds
  public static final double kClimbUpSpeed = 0.8;    // Speed when climbing up
  public static final double kClimbDownSpeed = -0.5; // Speed when climbing down

  // Motor configuration
  public static final SparkMaxConfig config = new SparkMaxConfig();

  static {
    config
        .idleMode(IdleMode.kBrake)  // Use brake mode to hold position
        .smartCurrentLimit(kMotorCurrentLimit)
        .inverted(kMotorInverted);
  }
}

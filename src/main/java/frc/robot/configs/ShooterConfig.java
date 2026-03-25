package frc.robot.configs;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.ShooterConstants;

public final class ShooterConfig {
  public static final SparkFlexConfig shooterConfig = new SparkFlexConfig();
  public static final SparkFlexConfig indexerConfig = new SparkFlexConfig();

  static {
    shooterConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kMotorCurrentLimit)
        .closedLoop
          .pid(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD)
          .velocityFF(ShooterConstants.kShooterFF);

    indexerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kMotorCurrentLimit)
        .closedLoop
          .pid(ShooterConstants.kIndexerP, ShooterConstants.kIndexerI, ShooterConstants.kIndexerD)
          .velocityFF(ShooterConstants.kIndexerFF);
  }
}

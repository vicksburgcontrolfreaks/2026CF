package frc.robot.configs;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.ShooterConstants;

public final class ShooterConfig {
  public static final SparkFlexConfig config = new SparkFlexConfig();
  public static final SparkFlexConfig indexerManualConfig = new SparkFlexConfig();

  static {
    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kMotorCurrentLimit)
        .closedLoop
          .pid(ShooterConstants.kVelocityP, ShooterConstants.kVelocityI, ShooterConstants.kVelocityD)
          .velocityFF(ShooterConstants.kVelocityFF);

    indexerManualConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.kIndexerManualCurrentLimit)
        .closedLoop
          .pid(ShooterConstants.kVelocityP, ShooterConstants.kVelocityI, ShooterConstants.kVelocityD)
          .velocityFF(ShooterConstants.kVelocityFF);
  }
}

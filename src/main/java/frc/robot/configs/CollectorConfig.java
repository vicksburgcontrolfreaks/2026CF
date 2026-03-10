package frc.robot.configs;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.CollectorConstants;

public final class CollectorConfig {
  public static final SparkFlexConfig collectorConfig = new SparkFlexConfig();
  public static final SparkMaxConfig hopperConfig = new SparkMaxConfig();

  static {
    collectorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(CollectorConstants.kMotorCurrentLimit);

    hopperConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(CollectorConstants.kMotorCurrentLimit)
        .closedLoop
          .pid(CollectorConstants.kHopperP, CollectorConstants.kHopperI, CollectorConstants.kHopperD)
          .outputRange(-CollectorConstants.kHopperSpeed, CollectorConstants.kHopperSpeed);
    hopperConfig.encoder
        .positionConversionFactor(1.0);
  }
}

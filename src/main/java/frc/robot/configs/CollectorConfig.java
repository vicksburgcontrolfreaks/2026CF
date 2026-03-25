package frc.robot.configs;

import com.revrobotics.spark.FeedbackSensor;
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
  }

  static {
    hopperConfig
        .idleMode(IdleMode.kCoast)

      .absoluteEncoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    hopperConfig
      .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.5, 0.0, 0.0)
        .outputRange(-0.1, 0.1)
        .maxMotion
          .maxVelocity(50)
          .maxAcceleration(100);
      
  }
}

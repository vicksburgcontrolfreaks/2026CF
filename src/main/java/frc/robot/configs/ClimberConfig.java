package frc.robot.configs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.ClimberConstants;

public final class ClimberConfig {
  public static final SparkMaxConfig config = new SparkMaxConfig();

  static {
    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ClimberConstants.kMotorCurrentLimit);

    config.closedLoop
          .pid(ClimberConstants.kPositionP, ClimberConstants.kPositionI, ClimberConstants.kPositionD);
  }
}

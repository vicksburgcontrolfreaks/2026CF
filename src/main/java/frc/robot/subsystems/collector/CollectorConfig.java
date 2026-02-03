package frc.robot.subsystems.collector;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class CollectorConfig {
  //collector 4 motors / shooter 2 opposite motors / lift 2 motors
  // CAN IDs for collector motors - UPDATE THESE TO MATCH YOUR ROBOT 
  public static final int kLeftMotorId = 10;
  public static final int kRightMotorId = 11;

  //CAN IDs for conveyor motor (feed balls into shooter)
  // public static final int kPlaceHolderId = 12;

  // CAN IDs for deployment motors (move collector on rail)
  public static final int kLeftDeploymentMotorId = 13;
  public static final int kRightDeploymentMotorId = 14;

  // Current limits
  public static final int kMotorCurrentLimit = 40; // Amps for collection motors
  public static final int kDeploymentMotorCurrentLimit = 30; // Amps for deployment motors

  // Motor inversions
  public static final boolean kLeftMotorInverted = false;
  public static final boolean kRightMotorInverted = true; // Typically opposite side motors are inverted
  public static final boolean kLeftDeploymentMotorInverted = false;
  public static final boolean kRightDeploymentMotorInverted = true;

  // Collector speeds
  public static final double kCollectionSpeed = 0.8; // Speed when collecting
  public static final double kDeploymentSpeed = 0.5; // Speed to move along rail

  // Position limits (encoder rotations)
  public static final double kDeployedPosition = 10.0;   // Rotations for deployed position
  public static final double kRetractedPosition = 0.0;   // Rotations for retracted position
  public static final double kPositionTolerance = 0.5;   // Tolerance in rotations

  // Motor configuration for collection motors
  public static final SparkMaxConfig collectionConfig = new SparkMaxConfig();

  // Motor configuration for deployment motors
  public static final SparkMaxConfig deploymentConfig = new SparkMaxConfig();

  static {
    collectionConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kMotorCurrentLimit)
        .inverted(kLeftMotorInverted);

    deploymentConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kDeploymentMotorCurrentLimit);
  }
}

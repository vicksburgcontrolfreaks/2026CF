package frc.robot.configs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.ModuleConstants;

public final class MAXSwerveModuleConfig {
  public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
  public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

  static {
    double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
        / ModuleConstants.kDrivingMotorReduction;
    double turningFactor = 2 * Math.PI;
    double nominalVoltage = 12.0;
    double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

    drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);
    drivingConfig.encoder
        .positionConversionFactor(drivingFactor) // meters
        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
    drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.1, 0, 0)
        .outputRange(-1, 1)
        .feedForward.kV(drivingVelocityFeedForward);

    turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of the steering motor
    turningConfig.absoluteEncoder
        .inverted(true)
        .positionConversionFactor(turningFactor) // radians
        .velocityConversionFactor(turningFactor / 60.0) // radians per second
        .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

    // Enable PID wrap around for the turning motor
    turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(1, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, turningFactor);
  }
}

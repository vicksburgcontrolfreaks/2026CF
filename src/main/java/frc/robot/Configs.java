package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ConfigConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class Shooter {
        public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();

        static {
            shooterConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40);
        }
    }

    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = ConfigConstants.kPositionConversionFactor;
            double nominalVoltage = ConfigConstants.kNominalVoltage;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor)
                    .velocityConversionFactor(drivingFactor * ConfigConstants.kVelocityConversionFactor);
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.04, 0, 0)
                    .outputRange(ConfigConstants.kMotorOutputMin, ConfigConstants.kMotorOutputMax)
                    .feedForward.kV(drivingVelocityFeedForward);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                    .inverted(true)
                    .positionConversionFactor(turningFactor)
                    .velocityConversionFactor(turningFactor * ConfigConstants.kVelocityConversionFactor)
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
}

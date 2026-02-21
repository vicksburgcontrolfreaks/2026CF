// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class NeoVortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class MotorModuleConstants {
    // MAXSwerve pinion options: 12T (~5.50:1), 13T (~5.08:1), 14T (~4.71:1)
    public static final int kDrivingMotorPinionTeeth = 14;

    public static final double kDrivingMotorFreeSpeedRps = NeoVortexMotorConstants.kFreeSpeedRpm / 60;

    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kNominalVoltage = 12.0;
    public static final double kMotorOutputMin = -1.0;
    public static final double kMotorOutputMax = 1.0;
    public static final double kVelocityConversionFactor = 1.0 / 60.0;
    public static final double kPositionConversionFactor = 2.0 * Math.PI;
  }

  public static class TelemetryConstants {
    public static final int kTelemetryUpdatePeriod = 5;  
  }

  @SuppressWarnings("removal")
  public static class ShooterConstants {
    public static final int kRightShooterId = 16;
    public static final int kFloorMotorId = 13;
    public static final int kIndexerMotorId = 14;
    public static final int kLeftShooterId = 19;

    public static final int kMotorCurrentLimit = 60;

    public static final double kVelocityP = 0.001;
    public static final double kVelocityI = 0.0;
    public static final double kVelocityD = 0.0;
    public static final double kVelocityFF = 1.0 / NeoVortexMotorConstants.kFreeSpeedRpm; // ~0.000147
    public static final double kTargetRPM = 3000; // Default/fallback RPM value
    // max rpm 6784

    // Shooter velocity lookup table: distance (meters) -> RPM
    // TODO: Replace these example values with actual tested values
    public static final double[][] kShooterVelocityTable = {
      // {distance in meters, RPM}
      {1.0, 2500},  // Very close shot
      {2.0, 3000},  // Close shot
      {3.0, 3500},  // Medium shot
      {4.0, 4000},  // Medium-far shot
      {5.0, 4500},  // Far shot
      {6.0, 5000}   // Very far shot
    };

    // Default RPM when distance is unavailable
    public static final double kDefaultRPM = 3000;

    public static final SparkFlexConfig config = new SparkFlexConfig();

    static {
      config
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(kMotorCurrentLimit)
          .closedLoop
            .pid(kVelocityP, kVelocityI, kVelocityD)
            .velocityFF(kVelocityFF);
    }

    /**
     * Get shooter RPM for a given distance to target using linear interpolation
     * @param distance Distance to target in meters
     * @return Target RPM for the shooter wheels
     */
    public static double getRPMForDistance(double distance) {
      // If distance is invalid, return default
      if (distance <= 0) {
        return kDefaultRPM;
      }

      // If distance is before first entry, use first RPM
      if (distance <= kShooterVelocityTable[0][0]) {
        return kShooterVelocityTable[0][1];
      }

      // If distance is beyond last entry, use last RPM
      if (distance >= kShooterVelocityTable[kShooterVelocityTable.length - 1][0]) {
        return kShooterVelocityTable[kShooterVelocityTable.length - 1][1];
      }

      // Linear interpolation between two nearest points
      for (int i = 0; i < kShooterVelocityTable.length - 1; i++) {
        double dist1 = kShooterVelocityTable[i][0];
        double dist2 = kShooterVelocityTable[i + 1][0];

        if (distance >= dist1 && distance <= dist2) {
          double rpm1 = kShooterVelocityTable[i][1];
          double rpm2 = kShooterVelocityTable[i + 1][1];

          // Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
          double ratio = (distance - dist1) / (dist2 - dist1);
          return rpm1 + ratio * (rpm2 - rpm1);
        }
      }

      return kDefaultRPM;
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismControllerPort = 1;

    public static final double kDeadband = 0.1;

    public static final double kNormalSpeedLimit = 0.4;
    public static final double kTurboSpeedLimit = 0.9;
    public static final double kPrecisionSpeedLimit = 0.2;
  }

  public static class SwerveConstants {
    public static final int kFrontLeftDriveMotorId = 2;
    public static final int kFrontLeftSteerMotorId = 3;

    public static final int kFrontRightDriveMotorId = 4;
    public static final int kFrontRightSteerMotorId = 5;

    public static final int kBackLeftDriveMotorId = 8;
    public static final int kBackLeftSteerMotorId = 9;

    public static final int kBackRightDriveMotorId = 6;
    public static final int kBackRightSteerMotorId = 7;

    public static final int kGyroCalibrationTimeSec = 2;

    public static final boolean m_fieldOriented = true;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDriveGearRatio = (45.0 * 22) / (14 * 15);
    public static final double kSteerGearRatio = 150.0 / 7.0;

    public static final int kDriveMotorCurrentLimit = 50;
    public static final int kSteerMotorCurrentLimit = 20;

    public static final double kRobotMassKg = 25.0; // Robot mass in kilograms
    public static final double kRobotMOI = 6.0; // Robot moment of inertia (kg*m²)
    public static final double kWheelCoefficientOfFriction = 1.2; // Wheel coefficient of friction
    public static final int kNumMotorsPerModule = 1; 

    public static final double kDriveP = 0.04;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveFF = 1.565;

    public static final double kSteerP = 1.0;
    public static final double kSteerI = 0.0;
    public static final double kSteerD = 0.0;

    public static final double kTrackWidthMeters = Units.inchesToMeters(18.5);
    public static final double kWheelBaseMeters = Units.inchesToMeters(18.5);

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
      new Translation2d(kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),
      new Translation2d(-kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
      new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0)
    );

    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;

    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    public static final double kEncoderNormalization = 2.0 * Math.PI;
    public static final double kGyroWrapModulo = 360.0;

    public static final double kXFormationAngleFrontLeft = 45.0;
    public static final double kXFormationAngleFrontRight = -45.0;
    public static final double kXFormationAngleBackLeft = -45.0;
    public static final double kXFormationAngleBackRight = 45.0;

    public static final double kYawCorrectionThresholdDegrees = 3.0;
    public static final double kYawCorrectionToleranceDegrees = 0.3;
    public static final double kYawCorrectionMaxPower = 0.07;
    public static final double kYawCorrectionMaxPowerLateral = 0.09;

    public static final double kTranslationSlewRate = 3.0;
    public static final double kRotationSlewRate = 3.0;

    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
        double drivingFactor = MotorModuleConstants.kWheelDiameterMeters * Math.PI
                / MotorModuleConstants.kDrivingMotorReduction;
        double turningFactor = MotorModuleConstants.kPositionConversionFactor;
        double nominalVoltage = MotorModuleConstants.kNominalVoltage;
        double drivingVelocityFeedForward = nominalVoltage / MotorModuleConstants.kDriveWheelFreeSpeedRps;
        drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
        drivingConfig.encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor * MotorModuleConstants.kVelocityConversionFactor);
        drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .outputRange(MotorModuleConstants.kMotorOutputMin, MotorModuleConstants.kMotorOutputMax)
                .feedForward.kV(drivingVelocityFeedForward);
        turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
        turningConfig.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(turningFactor)
                .velocityConversionFactor(turningFactor * MotorModuleConstants.kVelocityConversionFactor)
                .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
        turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static class PhotonVisionConstants {
    public static final String kCameraFrontName = "camera-front";
    public static final String kCameraBackName = "camera-back";
    public static final String kCameraLeftName = "camera-left";
    public static final String kCameraRightName = "camera-right";

    public static final Transform3d kRobotToFrontCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-4), Units.inchesToMeters(0.5), Units.inchesToMeters(14)),
      new Rotation3d(0, Math.toRadians(6), 0)
    );

    public static final Transform3d kRobotToBackCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-10), Units.inchesToMeters(0.5), Units.inchesToMeters(14)),
      new Rotation3d(0, Math.toRadians(6), Math.toRadians(180))
    );

    public static final Transform3d kRobotToLeftCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-7.5), Units.inchesToMeters(4.0), Units.inchesToMeters(14)),
      new Rotation3d(0, Math.toRadians(6), Math.toRadians(90))
    );

    public static final Transform3d kRobotToRightCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-7.5), Units.inchesToMeters(-3.0), Units.inchesToMeters(14)),
      new Rotation3d(0, Math.toRadians(6), Math.toRadians(-90))
    );

    public static final double[] kSingleTagStdDevs = {1.5, 1.5, 3.0};
    public static final double[] kMultiTagStdDevs = {0.5, 0.5, 1.0};

    public static final double kMaxAmbiguity = 0.2;
    public static final double kMaxTagDistance = 5.0;
    public static final int kMinTagsForHighConfidence = 2;

    public static final double kDistanceFactorThreshold = 2.0;
    public static final double kConfidenceTagCountMultiplier = 10.0;
    public static final double kConfidenceAmbiguityMultiplier = 5.0;
  }


  public static class AutoConstants {
    public static final double kPTranslation = 5.0;
    public static final double kITranslation = 0.0;
    public static final double kDTranslation = 0.0;

    public static final double kPRotation = 15.0;
    public static final double kIRotation = 0.25;  // Small I term to eliminate steady-state error
    public static final double kDRotation = 0.1;  // Small D term to reduce oscillation

    // PID constants for auto-align to target (separate from general rotation)
    public static final double kPRotationAutoAlign = 7.5;
    public static final double kIRotationAutoAlign = 0.0;
    public static final double kDRotationAutoAlign = 0.3;

    public static final double kBlueTargetX = 4.639;
    public static final double kBlueTargetY = 4.02;
    public static final double kRedTargetX = 11.942;
    public static final double kRedTargetY = 4.02;
  }

  public static class CollectorConstants {
    public static final int kUpperCollectorMotorId = 10;
    public static final int kLowerCollectorMotorId = 11;
    public static final int kHopperMotorId = 12;

    public static final int kMotorCurrentLimit = 40;

    public static final double kCollectorSpeed = 0.1;
    public static final double kHopperMaxSpeed = 0.5;
    public static final double kHopperGearRatio = 81.0;

    public static final double kHopperRetractedPosition = 0.0;
    public static final double kHopperDeployedPosition = 5.0;
    public static final double kHopperPositionTolerance = 0.1;

    public static final double kHopperP = 0.1;
    public static final double kHopperI = 0.0;
    public static final double kHopperD = 0.0;

    public static final SparkFlexConfig collectorConfig = new SparkFlexConfig();
    public static final SparkMaxConfig hopperConfig = new SparkMaxConfig();

    static {
      collectorConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(kMotorCurrentLimit);

      hopperConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(kMotorCurrentLimit)
          .closedLoop
            .pid(kHopperP, kHopperI, kHopperD)
            .outputRange(-kHopperMaxSpeed, kHopperMaxSpeed);
      hopperConfig.encoder
          .positionConversionFactor(1.0);
    }
  }

  public static class LEDConstants {
    public static final int kLEDPort = 0;
    public static final int kLEDCount = 23;

    public static final int kBlinkSpeed = 500;
    public static final int kBreatheSpeed = 50;

    public static final int kRainbowHueIncrement = 3;
    public static final int kRainbowHueModulo = 180;
    public static final int kChaseTailLength = 3;
    public static final double kChaseBrightnessFade = 0.3;

    public static final int kCycleCounterModulo = 4;

    public static final int kStartupTestR = 255;
    public static final int kStartupTestG = 255;
    public static final int kStartupTestB = 255;

    public static final int[] kOffColor = {0, 0, 0};
    public static final int[] kRedColor = {255, 0, 0};
    public static final int[] kGreenColor = {0, 255, 0};
    public static final int[] kBlueColor = {0, 0, 255};
    public static final int[] kYellowColor = {255, 255, 0};
    public static final int[] kPurpleColor = {128, 0, 128};
    public static final int[] kOrangeColor = {255, 165, 0};
    public static final int[] kWhiteColor = {255, 255, 255};

    public static final int[] kAllianceRedColor = {255, 0, 0};
    public static final int[] kAllianceBlueColor = {0, 0, 255};

    public static final int[] kDisabledColor = {255, 165, 0};
    public static final int[] kAutonomousColor = {0, 255, 0};
    public static final int[] kTeleopColor = {0, 0, 255};
    public static final int[] kErrorColor = {255, 0, 0};

    public static final int[] kAprilTagDetectedColor = {0, 255, 0};
    public static final int[] kAprilTagMultipleColor = {255, 255, 0};
    public static final int[] kAprilTagAlignedColor = {0, 255, 255};
  }
}

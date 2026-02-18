// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  public static final class ModuleConstants {
    // MAXSwerve pinion options: 12T (~5.50:1), 13T (~5.08:1), 14T (~4.71:1)
    public static final int kDrivingMotorPinionTeeth = 14;

    public static final double kDrivingMotorFreeSpeedRps = NeoVortexMotorConstants.kFreeSpeedRpm / 60;

    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static class ShooterConstants {
    public static final int kTopMotorId = 0;
    public static final int kBottomMotorId = 1;
    public static final int kIndexerMotorId = 15;
    public static final int kMotorCurrentLimit = 40;

    public static final boolean kTopMotorInverted = false;
    public static final boolean kBottomMotorInverted = false;

    public static final double kTopShooterSpeed = 0.8;
    public static final double kFrontShooterStartPower = 0.2;
    public static final double kFrontShooterMaxSpeed = 0.8;

    public static final double kPShooter = 0.01;

    public static final double kMinShooterDistance = 1.0;
    public static final double kMaxShooterDistance = 5.0;
    public static final double kMinShooterPower = 0.7;
    public static final double kMaxShooterPower = 0.9;

    public static final double kDefaultShooterSpeed = 0.8;
    public static final double kSlowShooterSpeed = 0.5;
    public static final double kMaxShooterSpeed = 1.0;

    public static final double kIndexerSpeed = 0.6;
    public static final double kSpinUpTimeSeconds = 0.5;

    // Motor configuration for top shooter motor
    public static final SparkMaxConfig topMotorConfig = new SparkMaxConfig();

    // Motor configuration for bottom shooter motor
    public static final SparkMaxConfig bottomMotorConfig = new SparkMaxConfig();

    static {
      // Configure top motor
      topMotorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(kMotorCurrentLimit)
          .inverted(kTopMotorInverted);

      // Configure bottom motor
      bottomMotorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(kMotorCurrentLimit)
          .inverted(kBottomMotorInverted);
    }
  }

  public static class CollectorConstants {
    public static final int kLeftMotorId = 10;
    public static final int kRightMotorId = 11;

    public static final int kLeftDeploymentMotorId = 13;
    public static final int kRightDeploymentMotorId = 14;

    public static final int kMotorCurrentLimit = 40;
    public static final int kDeploymentMotorCurrentLimit = 30;

    public static final boolean kLeftMotorInverted = false;
    public static final boolean kRightMotorInverted = true;
    public static final boolean kLeftDeploymentMotorInverted = false;
    public static final boolean kRightDeploymentMotorInverted = true;

    public static final double kCollectionSpeed = 0.8;
    public static final double kDeploymentSpeed = 0.5;

    public static final double kDeployedPosition = 10.0;
    public static final double kRetractedPosition = 0.0;
    public static final double kPositionTolerance = 0.5;

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

  public static class ClimberConstants {
    public static final int kMotorId = 12;

    public static final int kMotorCurrentLimit = 40;
    public static final int kMotorStallLimit = 60;

    public static final boolean kMotorInverted = false;

    public static final double kExtendSpeed = 0.8;
    public static final double kRetractSpeed = -0.8;
    public static final double kSlowSpeed = 0.3;
    public static final double kClimbUpSpeed = 0.8;
    public static final double kClimbDownSpeed = -0.5;

    public static final double kGearRatio = 60.0;

    // Motor configuration
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(kMotorCurrentLimit)
          .inverted(kMotorInverted);
    }
  }

  public static class TestMotorConstants {
    public static final int kRightShooterId = 16;
    public static final int kFloorMotorId = 17;
    public static final int kIndexerMotorId = 18;
    public static final int kLeftShooterId = 19;

    public static final int kMotorCurrentLimit = 60;

    // Velocity control constants
    public static final double kVelocityP = 0.001;
    public static final double kVelocityI = 0.0;
    public static final double kVelocityD = 0.0;
    public static final double kVelocityFF = 1.0 / NeoVortexMotorConstants.kFreeSpeedRpm; // ~0.000147
    public static final double kTargetRPM = 3000; // 40% of max velocity
    // max rpm 6784 

    // Motor configuration for SparkFlex
    public static final SparkFlexConfig config = new SparkFlexConfig();

    static {
      config
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(kMotorCurrentLimit)
          .closedLoop
            .pid(kVelocityP, kVelocityI, kVelocityD)
            .velocityFF(kVelocityFF);
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismControllerPort = 1;
    public static final int kJoystickPort = 0;

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

    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDriveGearRatio = (45.0 * 22) / (14 * 15);
    public static final double kSteerGearRatio = 150.0 / 7.0;

    public static final boolean kDriveMotorInverted = false;
    public static final boolean kSteerMotorInverted = true;
    public static final boolean kSteerEncoderInverted = true;

    public static final int kDriveMotorCurrentLimit = 50;
    public static final int kSteerMotorCurrentLimit = 20;

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
  }


  public static class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 6.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPXController = 1.2;
    public static final double kPYController = 1.2;
    public static final double kPThetaController = 1.2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double kPForward = 1.0;
    public static final double kPLateral = 1.5;

    public static final double kForwardTolerance = Units.inchesToMeters(1.0);
    public static final double kLateralTolerance = 1.0;

    public static final double kPTranslation = 5.0;
    public static final double kITranslation = 0.0;
    public static final double kDTranslation = 0.0;

    public static final double kPRotation = 5.0;
    public static final double kIRotation = 0.0;
    public static final double kDRotation = 0.0;

    // PID constants for auto-align to target (separate from general rotation)
    public static final double kPRotationAutoAlign = 7.5;
    public static final double kIRotationAutoAlign = 0.0;
    public static final double kDRotationAutoAlign = 0.3;

    public static final double kMaxAutoSpeedMetersPerSecond = 3.0;
    public static final double kMaxAutoAngularSpeedRadiansPerSecond = Math.PI;

    public static final double kBlueTargetX = 4.639;
    public static final double kBlueTargetY = 4.02;
    public static final double kRedTargetX = 11.942;
    public static final double kRedTargetY = 4.02;
  }

  public static class LEDConstants {
    public static final int kLEDPort = 0;
    public static final int kLEDCount = 23;

    public static final int kBlinkSpeed = 500;
    public static final int kBreatheSpeed = 50;
    public static final int kRainbowSpeed = 100;

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

  public static class RobotContainerConstants {
    public static final boolean kUseJoystick = false;

    public static final double kCollectorFullSpeed = 1.0;
    public static final double kCollectorHalfSpeed = 0.5;
    public static final double kCollectorFullSpeedReverse = -1.0;
    public static final double kCollectorHalfSpeedReverse = -0.5;

    public static final double kSetXTimeoutSeconds = 1.0;

    public static final double kTriggerThreshold = 0.1;

    public static final double kThrottleLowerThreshold = -0.3;
    public static final double kThrottleUpperThreshold = 0.3;
  }

  public static class DriveCommandConstants {
    public static final double kTranslationSlewRate = 3.0;
    public static final double kRotationSlewRate = 3.0;

    // Yaw correction constants
    public static final double kYawCorrectionThresholdDegrees = 3.0;  // Activates at 3 degrees
    public static final double kYawCorrectionToleranceDegrees = 0.3;  // Stops within 0.3 degrees
    public static final double kYawCorrectionMaxPower = 0.07;         // 7% power for forward/back
    public static final double kYawCorrectionMaxPowerLateral = 0.09;  // 9% power for left/right
  }

  public static class SwerveDriveConstants {
    public static final int kTelemetryUpdatePeriod = 5;

    public static final double kXFormationAngleFrontLeft = 45.0;
    public static final double kXFormationAngleFrontRight = -45.0;
    public static final double kXFormationAngleBackLeft = -45.0;
    public static final double kXFormationAngleBackRight = 45.0;

    public static final double kGyroWrapModulo = 360.0;

    public static final double kEncoderNormalization = 2.0 * Math.PI;

    public static final String kTelemetryTableName = "SwerveDrive";
    public static final String kFieldOrientedTopic = "FieldOriented";
    public static final String kGyroAngleTopic = "GyroAngle";
    public static final String kPoseXTopic = "PoseX";
    public static final String kPoseYTopic = "PoseY";
    public static final String kPoseRotationTopic = "PoseRotation";
    public static final String kFrontLeftAngleTopic = "FrontLeftAngle";
    public static final String kFrontRightAngleTopic = "FrontRightAngle";
    public static final String kBackLeftAngleTopic = "BackLeftAngle";
    public static final String kBackRightAngleTopic = "BackRightAngle";
    public static final String kFrontLeftVelocityTopic = "FrontLeftVelocity";
    public static final String kFrontRightVelocityTopic = "FrontRightVelocity";
    public static final String kBackLeftVelocityTopic = "BackLeftVelocity";
    public static final String kBackRightVelocityTopic = "BackRightVelocity";
  }

  public static class ConfigConstants {
    // Motor nominal voltage
    public static final double kNominalVoltage = 12.0;

    // Motor output ranges
    public static final double kMotorOutputMin = -1.0;
    public static final double kMotorOutputMax = 1.0;

    // Velocity conversion factor (RPM to radians per second)
    public static final double kVelocityConversionFactor = 1.0 / 60.0;

    // Position conversion factor
    public static final double kPositionConversionFactor = 2.0 * Math.PI;
  }

  public static class HopperConstants {
    public static final int kMotorId = 20;

    public static final int kMotorCurrentLimit = 30;

    public static final boolean kMotorInverted = false;

    public static final double kGearRatio = 81.0;

    public static final double kDeploySpeed = 0.1;
    public static final double kRetractSpeed = -0.1;

    public static final SparkFlexConfig config = new SparkFlexConfig();

    static {
      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(kMotorCurrentLimit)
          .inverted(kMotorInverted);
    }
  }

  public static class SwerveConfigConstants {
    // Robot physical properties
    public static final double kRobotMassKg = 25.0; // Robot mass in kilograms
    public static final double kRobotMOI = 6.0; // Robot moment of inertia (kg*m²)
    public static final double kWheelCoefficientOfFriction = 1.2; // Wheel coefficient of friction

    // Drive system configuration
    public static final int kNumMotorsPerModule = 1; // Number of drive motors per module
  }

  public static class VisionConstants {
    public static final double kBlueAllianceXThreshold = 2.0;
    public static final double kRedAllianceXThreshold = 14.54;

    public static final double kDistanceFactorThreshold = 2.0;
    public static final double kConfidenceTagCountMultiplier = 10.0;
    public static final double kConfidenceAmbiguityMultiplier = 5.0;

    public static final String kTelemetryTableName = "PhotonVision";
    public static final String kHasTargetTopic = "HasTarget";
    public static final String kTargetYawTopic = "TargetYaw";
    public static final String kTargetPitchTopic = "TargetPitch";
    public static final String kTargetAreaTopic = "TargetArea";
    public static final String kTargetSkewTopic = "TargetSkew";
    public static final String kPoseXTopic = "PoseX";
    public static final String kPoseYTopic = "PoseY";
    public static final String kPoseRotationTopic = "PoseRotation";
    public static final String kBestTagIDTopic = "BestTagID";
    public static final String kBestTagDistanceTopic = "BestTagDistance";
    public static final String kTagCountTopic = "TagCount";
  }
}

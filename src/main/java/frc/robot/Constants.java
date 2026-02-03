// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
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
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoVortexMotorConstants.kFreeSpeedRpm / 60;

    public static final double kWheelDiameterMeters = 0.0762; // 3in
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static class ShooterConstants {
    // CAN IDs for shooter motors
    public static final int kTopMotorId = 0;
    public static final int kBottomMotorId = 1;
    public static final int kIndexerMotorId = 15;          // CAN ID for indexer motor

    // Current limits
    public static final int kMotorCurrentLimit = 40; // Amps

    // Motor inversions
    public static final boolean kTopMotorInverted = false;
    public static final boolean kBottomMotorInverted = false;

    // Shooter motor speeds (legacy - kept for compatibility)
    public static final double kTopShooterSpeed = 0.8;      // Top shooter runs at constant 80%
    public static final double kFrontShooterStartSpeed = 0.2; // Front shooter starts at 20%
    public static final double kFrontShooterMaxSpeed = 0.8;   // Front shooter caps at 80%

    // Proportional ramp value for front shooter (legacy)
    public static final double kPShooter = 0.01; // Increase front shooter by 1% per loop

    // Distance-based shooting parameters
    // Minimum distance (meters) - closer shots use minimum power
    public static final double kMinShooterDistance = 1.0;  // 1 meter
    // Maximum distance (meters) - farther shots use maximum power
    public static final double kMaxShooterDistance = 5.0;  // 5 meters
    // Minimum shooter power (0.0 to 1.0) for close-range shots
    public static final double kMinShooterPower = 0.7;     // 70%
    // Maximum shooter power (0.0 to 1.0) for long-range shots
    public static final double kMaxShooterPower = 0.9;     // 90%

    // Shooter speed settings
    public static final double kDefaultShooterSpeed = 0.8; // 80% power
    public static final double kSlowShooterSpeed = 0.5; // 50% power for close shots
    public static final double kMaxShooterSpeed = 1.0; // Maximum speed

    // Indexer motor configuration
    public static final double kIndexerSpeed = 0.6;        // Indexer speed (60%)
    public static final double kSpinUpTimeSeconds = 0.5;   // Time to wait for shooter to reach speed before running indexer

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
    // CAN IDs for collector motors
    public static final int kLeftMotorId = 10;
    public static final int kRightMotorId = 11;

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

  public static class ClimberConstants {
    // CAN ID for climber motor
    public static final int kMotorId = 12;

    // Current limits
    public static final int kMotorCurrentLimit = 40; // Amps - adjust based on your winch requirements
    public static final int kMotorStallLimit = 60; // Amps - higher limit for stall conditions

    // Motor inversion
    public static final boolean kMotorInverted = false;

    // Climber speeds (positive = extend, negative = retract)
    public static final double kExtendSpeed = 0.8; // Speed when extending climber
    public static final double kRetractSpeed = -0.8; // Speed when retracting climber
    public static final double kSlowSpeed = 0.3; // Slow speed for fine adjustments
    public static final double kClimbUpSpeed = 0.8;    // Speed when climbing up
    public static final double kClimbDownSpeed = -0.5; // Speed when climbing down

    // Gear ratio information (for reference)
    public static final double kGearRatio = 60.0; // 60:1 reduction

    // Motor configuration
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      config
          .idleMode(IdleMode.kBrake)  // Use brake mode to hold position
          .smartCurrentLimit(kMotorCurrentLimit)
          .inverted(kMotorInverted);
    }
  }

  public static class OperatorConstants {
    // Controller ports
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismControllerPort = 1;
    public static final int kJoystickPort = 0; // Logitech Extreme 3D Pro (same port as Xbox when switched)

    // Joystick deadband
    public static final double kDeadband = 0.1;

    // Drive speed limits (0.0 to 1.0)
    // Start with very conservative values for testing, increase as you gain confidence
    public static final double kNormalSpeedLimit = 0.025;  // ~0.7 m/s - safe walking speed
    public static final double kTurboSpeedLimit = 0.05;   // ~1.1 m/s - moderate speed
    public static final double kPrecisionSpeedLimit = 0.001; // ~0.2 m/s - very slow for precise movements
  }

  public static class SwerveConstants {
    // CAN IDs - UPDATE THESE TO MATCH YOUR ROBOT
    public static final int kFrontLeftDriveMotorId = 2;
    public static final int kFrontLeftSteerMotorId = 3;

    public static final int kFrontRightDriveMotorId = 4;
    public static final int kFrontRightSteerMotorId = 5;

    public static final int kBackLeftDriveMotorId = 8;
    public static final int kBackLeftSteerMotorId = 9;

    public static final int kBackRightDriveMotorId = 6;
    public static final int kBackRightSteerMotorId = 7;

    // ADIS16470 IMU Configuration
    // Connected via SPI (onboard port), no CAN ID needed
    // Using 2-second calibration time for good accuracy with faster startup (robot must be stationary during init)
    public static final int kGyroCalibrationTimeSec = 2;

    // Physical constants
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // Gear ratios for REV MAXSwerve - THESE ARE PLACEHOLDERS, MEASURE YOUR ACTUAL RATIOS
    // REV MAXSwerve drive ratios depend on pinion (12T=~5.50:1, 13T=~5.08:1, 14T=~4.71:1)
    // Current setting assumes 14T pinion (kDrivingMotorPinionTeeth = 14)
    public static final double kDriveGearRatio = (45.0 * 22) / (14 * 15); // Calculated from MAXSwerve gearing
    public static final double kSteerGearRatio = 150.0 / 7.0; // REV MAXSwerve steering ratio (9424:392 = ~21.43:1)

    // Motor inversions
    public static final boolean kDriveMotorInverted = false;
    public static final boolean kSteerMotorInverted = true;
    public static final boolean kSteerEncoderInverted = true;

    // Current limits (from REV MAXSwerve template)
    public static final int kDriveMotorCurrentLimit = 50; // Amps
    public static final int kSteerMotorCurrentLimit = 20; // Amps

    // PID Constants for drive motors (from REV MAXSwerve template)
    public static final double kDriveP = 0.04;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    // Feedforward: 12V / theoretical max speed (m/s)
    // Calculated: 12.0 / (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters / kDrivingMotorReduction)
    public static final double kDriveFF = 1.565; // REV MAXSwerve recommended value

    // PID Constants for steer motors (from REV MAXSwerve template)
    public static final double kSteerP = 1.0;
    public static final double kSteerI = 0.0;
    public static final double kSteerD = 0.0;

    // Chassis configuration - MEASURE YOUR ROBOT
    // Distance from robot center to module (front-back)
    public static final double kTrackWidthMeters = Units.inchesToMeters(18.5);
    // Distance from robot center to module (left-right)
    public static final double kWheelBaseMeters = Units.inchesToMeters(18.5);

    // Swerve kinematics
    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
      // Front right
      new Translation2d(kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),
      // Back left
      new Translation2d(-kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
      // Back right
      new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0)
    );

    // Maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5; // Theoretical max ~4.8 m/s for NEO
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2; // 1 rotation per second

    // Chassis angular offsets for swerve module positions
    // MEASURE THESE VALUES for your robot - angles in radians
    // These account for mechanical imperfections in module mounting
    // Set to 0.0 initially, then adjust if modules don't point straight forward when enabled
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2; // Radians
    public static final double kFrontRightChassisAngularOffset = 0; // Radians
    public static final double kBackLeftChassisAngularOffset = Math.PI; // Radians
    public static final double kBackRightChassisAngularOffset = Math.PI / 2; // Radians

    // Note: Encoder zero offsets are still configured on SparkMax via REV Hardware Client
    // These chassis offsets are ADDITIONAL corrections for mechanical alignment
  }

  public static class PhotonVisionConstants {
    // Camera names (must match names configured in PhotonVision web UI)
    public static final String kCameraFrontName = "camera-front";
    public static final String kCameraBackName = "camera-back";
    public static final String kCameraLeftName = "camera-left";
    public static final String kCameraRightName = "camera-right";

    // Camera transforms relative to robot center (robot-to-camera)
    // TODO: Measure and update these values for your robot!
    // Positive X = forward, Positive Y = left, Positive Z = up
    // Rotations: Roll (X), Pitch (Y), Yaw (Z) in radians

    // Front camera: mounted on front tower, facing forward
    public static final Transform3d kRobotToFrontCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-4), Units.inchesToMeters(0.5), Units.inchesToMeters(14)), // 12" forward, 10" up
      new Rotation3d(0, Math.toRadians(6), 0) // Pitched down 6°
    );

    // Back camera: mounted on watchtower structure (back center), facing backward
    public static final Transform3d kRobotToBackCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-10), Units.inchesToMeters(0.5), Units.inchesToMeters(14)), // 9.6" back, 0.5" centered, 19.4" up
      new Rotation3d(0, Math.toRadians(6), Math.toRadians(180)) // Pitched down 6°, facing back
    );

    // Left camera: mounted on left tower, facing left
    public static final Transform3d kRobotToLeftCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-7.5), Units.inchesToMeters(4.0), Units.inchesToMeters(14)), // 12" left, 10" up
      new Rotation3d(0, Math.toRadians(6), Math.toRadians(90)) // Pitched down 6°, facing left
    );

    // Right camera: mounted on right tower, facing right
    public static final Transform3d kRobotToRightCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-7.5), Units.inchesToMeters(-3.0), Units.inchesToMeters(14)), // 12" right, 10" up
      new Rotation3d(0, Math.toRadians(6), Math.toRadians(-90)) // Pitched down 6°, facing right
    );

    // Standard deviations for multi-camera pose estimation
    // Lower = trust more, higher = trust less
    // [x, y, rotation] in meters and radians

    // Single tag standard deviations
    public static final double[] kSingleTagStdDevs = {1.5, 1.5, 3.0};

    // Multi-tag standard deviations (when multiple tags visible)
    public static final double[] kMultiTagStdDevs = {0.5, 0.5, 1.0};

    // Filtering thresholds
    public static final double kMaxAmbiguity = 0.2; // Maximum pose ambiguity
    public static final double kMaxTagDistance = 5.0; // Maximum distance to trust tags (meters)
    public static final int kMinTagsForHighConfidence = 2; // Minimum tags for high confidence
  }


  public static class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 6.0; // 4.0
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0; // 1.0
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPXController = 1.2; // 1.0
    public static final double kPYController = 1.2; // 1.0
    public static final double kPThetaController = 1.2; // 1.0

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // PID gains for forward (distance) and lateral control (tuning these is
    // necessary).
    public static final double kPForward = 1.0; // Example value; adjust by testing
    public static final double kPLateral = 1.5; // Example value; adjust by testing

    // APRIL TAG: Tolerances to determine when alignment is good enough.

    public static final double kForwardTolerance = Units.inchesToMeters(1.0); // ~0.1524 m
    public static final double kLateralTolerance = 1.0; // Acceptable error in tx (in degrees)

    // PathPlanner PID constants
    public static final double kPTranslation = 5.0;
    public static final double kITranslation = 0.0;
    public static final double kDTranslation = 0.0;

    public static final double kPRotation = 5.0;
    public static final double kIRotation = 0.0;
    public static final double kDRotation = 0.0;

    // Max auto speeds
    public static final double kMaxAutoSpeedMetersPerSecond = 3.0;
    public static final double kMaxAutoAngularSpeedRadiansPerSecond = Math.PI;

    // Target position coordinates (meters)
    public static final double kBlueTargetX = 12.5;
    public static final double kBlueTargetY = 4.6;
    public static final double kRedTargetX = 5.2;
    public static final double kRedTargetY = 4.6;
  }

  public static class LEDConstants {
    // PWM port for LED strip on RoboRIO
    public static final int kLEDPort = 0;

    // Number of addressable LEDs in the strip
    public static final int kLEDCount = 23;

    // Animation speeds (in milliseconds)
    public static final int kBlinkSpeed = 500;
    public static final int kBreatheSpeed = 50;
    public static final int kRainbowSpeed = 100;

    // LED animation parameters
    public static final int kRainbowHueIncrement = 3; // Hue increment for rainbow animation
    public static final int kRainbowHueModulo = 180; // Hue wrapping value
    public static final int kChaseTailLength = 3; // Number of LEDs for chase tail effect
    public static final double kChaseBrightnessFade = 0.3; // Brightness reduction per tail segment

    // LED cycle counter limit
    public static final int kCycleCounterModulo = 4; // For cycling LED patterns

    // Startup test color (white)
    public static final int kStartupTestR = 255;
    public static final int kStartupTestG = 255;
    public static final int kStartupTestB = 255;

    // Predefined colors (RGB values 0-255)
    public static final int[] kOffColor = {0, 0, 0};
    public static final int[] kRedColor = {255, 0, 0};
    public static final int[] kGreenColor = {0, 255, 0};
    public static final int[] kBlueColor = {0, 0, 255};
    public static final int[] kYellowColor = {255, 255, 0};
    public static final int[] kPurpleColor = {128, 0, 128};
    public static final int[] kOrangeColor = {255, 165, 0};
    public static final int[] kWhiteColor = {255, 255, 255};

    // Alliance colors
    public static final int[] kAllianceRedColor = {255, 0, 0};
    public static final int[] kAllianceBlueColor = {0, 0, 255};

    // Robot state colors
    public static final int[] kDisabledColor = {255, 165, 0}; // Orange
    public static final int[] kAutonomousColor = {0, 255, 0}; // Green
    public static final int[] kTeleopColor = {0, 0, 255}; // Blue
    public static final int[] kErrorColor = {255, 0, 0}; // Red

    // AprilTag detection colors
    public static final int[] kAprilTagDetectedColor = {0, 255, 0}; // Green
    public static final int[] kAprilTagMultipleColor = {255, 255, 0}; // Yellow
    public static final int[] kAprilTagAlignedColor = {0, 255, 255}; // Cyan
  }

  public static class RobotContainerConstants {
    // Controller selection flag (true = Joystick, false = Xbox controller)
    public static final boolean kUseJoystick = false;

    // Collector speed multipliers
    public static final double kCollectorFullSpeed = 1.0;
    public static final double kCollectorHalfSpeed = 0.5;
    public static final double kCollectorFullSpeedReverse = -1.0;
    public static final double kCollectorHalfSpeedReverse = -0.5;

    // Timeouts
    public static final double kSetXTimeoutSeconds = 1.0; // Timeout for wheel lock command

    // Trigger threshold (deadband)
    public static final double kTriggerThreshold = 0.1;

    // Throttle position thresholds (for joystick)
    public static final double kThrottleLowerThreshold = -0.3;
    public static final double kThrottleUpperThreshold = 0.3;
  }

  public static class DriveCommandConstants {
    // Slew rate limiters for smooth control (units per second)
    public static final double kTranslationSlewRate = 3.0;
    public static final double kRotationSlewRate = 3.0;
  }

  public static class SwerveDriveConstants {
    // Telemetry update period (in cycles)
    public static final int kTelemetryUpdatePeriod = 5;

    // X-formation wheel lock angles (degrees)
    public static final double kXFormationAngleFrontLeft = 45.0;
    public static final double kXFormationAngleFrontRight = -45.0;
    public static final double kXFormationAngleBackLeft = -45.0;
    public static final double kXFormationAngleBackRight = 45.0;

    // Angle wrapping constant
    public static final double kGyroWrapModulo = 360.0;

    // Encoder normalization constant
    public static final double kEncoderNormalization = 2.0 * Math.PI;

    // NetworkTables telemetry
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

  public static class SwerveConfigConstants {
    // Robot physical properties
    public static final double kRobotMassKg = 25.0; // Robot mass in kilograms
    public static final double kRobotMOI = 6.0; // Robot moment of inertia (kg*m²)
    public static final double kWheelCoefficientOfFriction = 1.2; // Wheel coefficient of friction

    // Drive system configuration
    public static final int kNumMotorsPerModule = 1; // Number of drive motors per module
  }

  public static class VisionConstants {
    // Field boundary thresholds for pose validation
    // FRC 2026 field is 16.54m long. Blue at X=0, Red at X=16.54, midfield at X=8.27
    public static final double kBlueAllianceXThreshold = 2.0; // Blue alliance minimum X (reject if X < 2.0m)
    public static final double kRedAllianceXThreshold = 14.54; // Red alliance maximum X (reject if X > 14.54m)

    // Distance and confidence thresholds
    public static final double kDistanceFactorThreshold = 2.0; // Distance scaling for standard deviation
    public static final double kConfidenceTagCountMultiplier = 10.0; // Tag count confidence boost
    public static final double kConfidenceAmbiguityMultiplier = 5.0; // Ambiguity confidence boost

    // NetworkTables telemetry
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

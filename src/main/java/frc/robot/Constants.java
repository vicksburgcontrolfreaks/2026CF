// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class OperatorConstants {
    // Controller ports
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismControllerPort = 1;

    // Joystick deadband
    public static final double kDeadband = 0.1;

    // Drive speed limits (0.0 to 1.0)
    public static final double kNormalSpeedLimit = 0.8;
    public static final double kTurboSpeedLimit = 1.0;
    public static final double kPrecisionSpeedLimit = 0.3;
  }

  public static class SwerveConstants {
    // CAN IDs - UPDATE THESE TO MATCH YOUR ROBOT
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontLeftSteerMotorId = 2;

    public static final int kFrontRightDriveMotorId = 3;
    public static final int kFrontRightSteerMotorId = 4;

    public static final int kBackLeftDriveMotorId = 5;
    public static final int kBackLeftSteerMotorId = 6;

    public static final int kBackRightDriveMotorId = 7;
    public static final int kBackRightSteerMotorId = 8;

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

    // Current limits
    public static final int kDriveMotorCurrentLimit = 40; // Amps
    public static final int kSteerMotorCurrentLimit = 20; // Amps

    // PID Constants for drive motors
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveFF = 0.0;

    // PID Constants for steer motors
    public static final double kSteerP = 0.5;
    public static final double kSteerI = 0.0;
    public static final double kSteerD = 0.1;
    public static final double kSteerFF = 0.0;

    // Chassis configuration - MEASURE YOUR ROBOT
    // Distance from robot center to module (front-back)
    public static final double kTrackWidthMeters = Units.inchesToMeters(22.0);
    // Distance from robot center to module (left-right)
    public static final double kWheelBaseMeters = Units.inchesToMeters(22.0);

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

    // Module offsets are configured directly on SparkMax controllers via REV Hardware Client
    // No software offset constants needed
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

    // Front camera: mounted on front bumper, facing forward
    public static final Transform3d kRobotToFrontCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(12.0), 0.0, Units.inchesToMeters(10.0)), // 12" forward, 10" up
      new Rotation3d(0, Math.toRadians(-15), 0) // Pitched down 15°
    );

    // Back camera: mounted on back bumper, facing backward
    public static final Transform3d kRobotToBackCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-12.0), 0.0, Units.inchesToMeters(10.0)), // 12" back, 10" up
      new Rotation3d(0, Math.toRadians(-15), Math.toRadians(180)) // Pitched down 15°, facing back
    );

    // Left camera: mounted on left side, facing left
    public static final Transform3d kRobotToLeftCamera = new Transform3d(
      new Translation3d(0.0, Units.inchesToMeters(12.0), Units.inchesToMeters(10.0)), // 12" left, 10" up
      new Rotation3d(0, Math.toRadians(-15), Math.toRadians(90)) // Pitched down 15°, facing left
    );

    // Right camera: mounted on right side, facing right
    public static final Transform3d kRobotToRightCamera = new Transform3d(
      new Translation3d(0.0, Units.inchesToMeters(-12.0), Units.inchesToMeters(10.0)), // 12" right, 10" up
      new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-90)) // Pitched down 15°, facing right
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
  }
}

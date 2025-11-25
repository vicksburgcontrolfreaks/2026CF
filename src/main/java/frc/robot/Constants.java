// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
    // public static final double kDrivingMotorFreeSpeedRps =
    // NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kDrivingMotorFreeSpeedRps = NeoVortexMotorConstants.kFreeSpeedRpm / 60;

    public static final double kWheelDiameterMeters = 0.0762; // 3in
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kMechanismControllerPort = 1;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

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

    // ADIS16470 IMU is connected via SPI (onboard port), no CAN ID needed

    // Physical constants
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // Gear ratios - THESE ARE PLACEHOLDERS, MEASURE YOUR ACTUAL RATIOS
    // Common swerve ratios: SDS Mk4i L1=8.14:1, L2=6.75:1, L3=6.12:1
    public static final double kDriveGearRatio = 6.75; // Drive motor rotations per wheel rotation
    public static final double kSteerGearRatio = 150.0 / 7.0; // Steer motor rotations per module rotation

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

  public static class VisionConstants {
    public static final String kLimelightName = "limelight";

    // MegaTag2 Standard Deviations for Pose Estimation
    // Lower values = trust vision more, higher values = trust vision less
    // [x, y, rotation] in meters and radians

    // Single tag standard deviations - less confident
    public static final double[] kSingleTagStdDevs = {1.0, 1.0, 2.0};

    // Multi-tag (MegaTag2) standard deviations - more confident
    public static final double[] kMultiTagStdDevs = {0.5, 0.5, 1.0};

    // Fallback vision standard deviations (used if estimate is null)
    public static final double[] kVisionStdDevs = {0.7, 0.7, 999999}; // Don't trust rotation from vision

    // Filtering thresholds for vision measurements
    // Maximum ambiguity to accept vision measurements (0.0-1.0, lower = more strict)
    public static final double kMaxPoseAmbiguity = 0.2;

    // Maximum distance to accept AprilTag measurements (meters)
    public static final double kMaxDistanceMeters = 4.0;

    // Minimum tag area to accept measurements (percentage of image)
    // Tags too small in frame are likely far away or at extreme angles
    public static final double kMinTagArea = 0.01; // 1% of image area

    // Minimum tag span for multi-tag measurements (pixels)
    // Tags too close together provide poor geometry for pose estimation
    public static final double kMinTagSpan = 10.0; // pixels between tags
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

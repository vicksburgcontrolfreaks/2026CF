// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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

    // Module offsets (in rotations, 0-1)
    // These need to be calibrated! See calibration instructions below
    public static final double kFrontLeftOffset = 0.0;
    public static final double kFrontRightOffset = 0.0;
    public static final double kBackLeftOffset = 0.0;
    public static final double kBackRightOffset = 0.0;
  }

  public static class VisionConstants {
    public static final String kLimelightName = "limelight";

    // Standard deviations for pose estimation (increase = trust vision less)
    // [x, y, rotation] in meters and radians
    public static final double[] kVisionStdDevs = {0.7, 0.7, 999999}; // Don't trust rotation from vision

    // Maximum pose ambiguity to accept vision measurements (lower = more strict)
    public static final double kMaxPoseAmbiguity = 0.2;

    // Maximum distance to accept AprilTag measurements (meters)
    public static final double kMaxDistanceMeters = 4.0;
  }

  public static class AutoConstants {
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
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

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static class TelemetryConstants {
    public static final int kTelemetryUpdatePeriod = 5;  
  }

  @SuppressWarnings("removal")
  public static class ShooterConstants {
    public static final int kRightShooterId = 15;
    public static final int kFloorMotorId = 13;
    public static final int kIndexerMotorId = 14;
    public static final int kLeftShooterId = 16;

    public static final int kMotorCurrentLimit = 40;

    public static final double kVelocityP = 0.00045;
    public static final double kVelocityI = 0.00000025;
    public static final double kVelocityD = 0.000000;
    public static final double kVelocityFF = 3.7 / NeoVortexMotorConstants.kFreeSpeedRpm; // ~0.000147
    
    public static final double kShooterTargetRPM = 3200; // Flat RPM value
    // max rpm 6784

    public static final double kFloorMotorTargetRPM = -1700; // Floor motor target RPM
    public static final double kIndexerMotorTargetRPM = 2150; // Indexer motor target RPM

    /* DYNAMIC RPM LOOKUP TABLE - COMMENTED OUT
    // Shooter velocity lookup table: distance (meters) -> RPM
    // Replace these example values with actual tested values
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
    */

    public static final SparkFlexConfig config = new SparkFlexConfig();

    static {
      config
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(kMotorCurrentLimit)
          .closedLoop
            .pid(kVelocityP, kVelocityI, kVelocityD)
            .velocityFF(kVelocityFF);
    }

    /* DYNAMIC RPM CALCULATION METHOD - COMMENTED OUT
    /**
     * Get shooter RPM for a given distance to target using linear interpolation
     * @param distance Distance to target in meters
     * @return Target RPM for the shooter wheels
     */
    /*
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
    */
  }

  public static class ClimberConstants {
    public static final int kClimberMotorId  = 17;

    public static final int kMotorCurrentLimit = 40;

    // Position control PID constants
    public static final double kPositionP = 0.1;
    public static final double kPositionI = 0.0;
    public static final double kPositionD = 0.0;

    // Position setpoints (in rotations)
    // TODO: Tune these values based on your actual climber mechanism
    public static final double kRetractedPosition = 0.0;    // Fully retracted
    public static final double kExtendedPosition = 100.0;   // Fully extended

    // Position tolerance for checking if at target
    public static final double kPositionTolerance = 2.0;

    // Maximum speed for manual control
    public static final double kManualSpeed = 0.5;

    // Soft limits (in rotations)
    public static final double kSoftLimitMin = -5.0;   // Safety buffer below retracted
    public static final double kSoftLimitMax = 105.0;  // Safety buffer above extended

    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      config
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(kMotorCurrentLimit);

      config.closedLoop
            .pid(kPositionP, kPositionI, kPositionD);

      // Note: Soft limits are enforced in software in ClimberSubsystem.setSpeed()
      // Hardware soft limits can be configured via Phoenix Tuner if needed
    }
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismControllerPort = 1;
    public static final double kDriveDeadband = 0.05;

    public static final double kNormalSpeedLimit = 0.75;
    public static final double kTurboSpeedLimit = 0.9;
    public static final double kPrecisionSpeedLimit = 0.2;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs - PRESERVED FROM YOUR ORIGINAL CONFIGURATION
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;
  }

  public static class PhotonVisionConstants {
    public static final String kCameraFrontName = "camera-front";
    public static final String kCameraBackName = "camera-back";
    public static final String kCameraLeftName = "camera-left";
    public static final String kCameraRightName = "camera-right";

    // Translation3d(X, Y, Z):
    //   X: forward(+)/back(-) from robot center (in inches, converted to meters)
    //   Y: left(+)/right(-) from robot center
    //   Z: up(+)/down(-) from robot center
    // Rotation3d(roll, pitch, yaw):
    //   roll: rotation around X-axis (tilt left/right) in radians
    //   pitch: rotation around Y-axis (tilt up/down) in radians
    //   yaw: rotation around Z-axis (spin left/right) in radians - 0° = forward, 90° = left, 180° = back, -90° = right
    public static final Transform3d kRobotToFrontCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-5.771), Units.inchesToMeters(0), Units.inchesToMeters(28.15)), 
      new Rotation3d(0, Math.toRadians(6), Math.toRadians(0)) // roll: 0°, pitch: 6°, yaw: 0°
    );

    public static final Transform3d kRobotToBackCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(-8.845), Units.inchesToMeters(7.60), Units.inchesToMeters(26.566)), 
      new Rotation3d(0, Math.toRadians(6), Math.toRadians(180)) // roll: 0°, pitch: 6°, yaw: 180°
    );

    public static final Transform3d kRobotToLeftCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(2.89), Units.inchesToMeters(-13.75), Units.inchesToMeters(24.61)), 
      new Rotation3d(0, Math.toRadians(0), Math.toRadians(90)) // roll: 0°, pitch: 6°, yaw: 90°
    );

    public static final Transform3d kRobotToRightCamera = new Transform3d(
      new Translation3d(Units.inchesToMeters(2.89), Units.inchesToMeters(13.75), Units.inchesToMeters(24.61)),
      new Rotation3d(0, Math.toRadians(0), Math.toRadians(-90)) // roll: 0°, pitch: 6°, yaw: -90°
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


  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Legacy PathPlanner constants (kept for compatibility)
    public static final double kPTranslation = 5.0;
    public static final double kITranslation = 0.0;
    public static final double kDTranslation = 0.0;

    public static final double kPRotation = 15.0;
    public static final double kIRotation = 0.25;
    public static final double kDRotation = 0.1;

    public static final Translation2d blueTarget = new Translation2d(4.639, 4.02);
    public static final Translation2d redTarget = new Translation2d(11.942, 4.02);

    // RotateToTarget constants
    public static final double kRotateToTargetP = 0.02;
    public static final double kRotateToTargetI = 0.0001;
    public static final double kRotateToTargetD = 0.003;
    public static final double kRotateToTargetTolerance = 2.0;
    public static final double kRotateToTargetMaxVelocity = 0.5;
  }

  public static class CollectorConstants {
    public static final int kUpperCollectorMotorId = 10;
    public static final int kLowerCollectorMotorId = 11;
    public static final int kHopperMotorId = 12;
    public static final int kLimitSwitchDIO = 0;

    public static final int kMotorCurrentLimit = 40;

    public static final double kCollectorSpeed = 0.2;
    public static final double kHopperSpeed = 0.3;

    // Pneumatic positions (from pneumatic's perspective)
    public static final double kHopperPneumaticExtendedPosition = -208.0;      // Spool unwound, pneumatic deployed
    public static final double kHopperPneumaticRetractedPosition = 0.0;   // Spool wound, pneumatic pulled back
    public static final double kHopperPositionTolerance = 1;

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
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(kMotorCurrentLimit)
          .closedLoop
            .pid(kHopperP, kHopperI, kHopperD)
            .outputRange(-kHopperSpeed, kHopperSpeed);
      hopperConfig.encoder
          .positionConversionFactor(1.0);
    }
  }

  public static class LEDConstants {
    // Hardware configuration
    public static final int kLEDPort = 0;
    public static final int kLEDCount = 23;

    // Color definitions (RGB values 0-255)
    public static final int[] kGreenColor = {0, 255, 0};
    public static final int[] kOffColor = {0, 0, 0};
  }
}

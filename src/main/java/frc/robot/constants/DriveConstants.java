package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  // Maximum capable speeds
  public static final double kMaxSpeedMetersPerSecond = 4.8;
  public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

  // Distance between centers of right and left wheels on robot
  public static final double kTrackWidth = Units.inchesToMeters(26.5);
  // Distance between front and back wheels on robot
  public static final double kWheelBase = Units.inchesToMeters(26.5);
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
  public static final double kFrontRightChassisAngularOffset = 0;
  public static final double kBackLeftChassisAngularOffset = Math.PI;
  public static final double kBackRightChassisAngularOffset = Math.PI / 2;

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

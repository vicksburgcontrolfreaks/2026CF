package frc.robot.constants;

public class ShooterConstants {
  public static final int kFloorMotorId = 13;

  public static final int kRIndexerMotorId = 14;
  public static final int kLIndexerMotorId = 15;
  public static final int kMIndexerMotorId = 16;

  public static final int kRightShooterId = 17;
  public static final int kLeftShooterId = 18;
  public static final int kMiddleShooterId = 19;

  public static final int kMotorCurrentLimit = 60;

  public static final double kShooterP = 0.00045;
  public static final double kShooterI = 0.00000025;
  public static final double kShooterD = 0.0001;
  public static final double kShooterFF = 3.0 / MotorConstants.NeoVortex.kFreeSpeedRpm;

  // Indexer PID values
  public static final double kIndexerP = 0.00045;
  public static final double kIndexerI = 0.00000025;
  public static final double kIndexerD = 0.0001;
  public static final double kIndexerFF = 3.0 / MotorConstants.NeoVortex.kFreeSpeedRpm;

  // max rpm 6784+
  public static final double kShooterTargetRPM = 3000;
  public static final double kFloorMotorTargetRPM = -1000;
  public static final double kIndexerMotorTargetRPM = 1000;

  // Shooter velocity lookup table: distance (meters) -> RPM
  // Reduced by ~18% to lower power output
  public static final double[][] kShooterVelocityTable = {
    {1.75, 2950},
    {2.69, 3450},
    {3.3, 4000},
  };
}

package frc.robot.constants;

@SuppressWarnings("removal")
public class ShooterConstants {
  public static final int kRightShooterId = 15;
  public static final int kFloorMotorId = 13;
  public static final int kIndexerMotorId = 14;
  public static final int kLeftShooterId = 16;
  public static final int kMotorCurrentLimit = 60;

  public static final double kVelocityP = 0.00045;
  public static final double kVelocityI = 0.00000025;
  public static final double kVelocityD = 0.000000;
  // max rpm 6784
  public static final double kVelocityFF = 3.7 / MotorConstants.NeoVortex.kFreeSpeedRpm;

  public static final double kShooterTargetRPM = 3200;

  public static final double kFloorMotorTargetRPM = -1200;
  public static final double kIndexerMotorTargetRPM = 2200;
  public static final int kIndexerManualCurrentLimit = 80;

  // Shooter velocity lookup table: distance (meters) -> RPM
  public static final double[][] kShooterVelocityTable = {
    {1.0, 2750},
    {2.0, 3250},
    {3.0, 3750},
    {4.0, 4250},
    {5.0, 4750},
    {6.0, 5250}
  };

  /**
   * Calculate target RPM based on distance to speaker using linear interpolation
   * @param distance Distance to speaker in meters
   * @return Target RPM for the shooter
   */
  public static double getRPMForDistance(double distance) {
    if (distance <= kShooterVelocityTable[0][0]) {
      return kShooterVelocityTable[0][1];
    }

    int lastIndex = kShooterVelocityTable.length - 1;
    if (distance >= kShooterVelocityTable[lastIndex][0]) {
      return kShooterVelocityTable[lastIndex][1];
    }

    for (int i = 0; i < kShooterVelocityTable.length - 1; i++) {
      double d1 = kShooterVelocityTable[i][0];
      double rpm1 = kShooterVelocityTable[i][1];
      double d2 = kShooterVelocityTable[i + 1][0];
      double rpm2 = kShooterVelocityTable[i + 1][1];

      if (distance >= d1 && distance <= d2) {
        return rpm1 + (rpm2 - rpm1) * (distance - d1) / (d2 - d1);
      }
    }

    return kShooterTargetRPM;
  }
}

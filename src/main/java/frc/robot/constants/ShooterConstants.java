package frc.robot.constants;

@SuppressWarnings("removal")
public class ShooterConstants {
  public static final int kFloorMotorId = 13;
  public static final int kRIndexerMotorId = 14;
  public static final int kLIndexerMotorId = 15;
  public static final int kMIndexerMotorId = 16;
  public static final int kRightShooterId = 17;
  public static final int kLeftShooterId = 18;
  public static final int kMiddleShooterId = 19;
  public static final int kMotorCurrentLimit = 60;

  public static final double kVelocityP = 0.00045;
  public static final double kVelocityI = 0.00000025;
  public static final double kVelocityD = 0.000000;
  // max rpm 6784
  public static final double kVelocityFF = 3.0 / MotorConstants.NeoVortex.kFreeSpeedRpm;

  public static final double kShooterTargetRPM = 3200;

  public static final double kFloorMotorTargetRPM = -1200;
  public static final double kIndexerMotorTargetRPM = 2200;
  public static final int kIndexerManualCurrentLimit = 80;

  // Shooter velocity lookup table: distance (meters) -> RPM
  // Reduced by ~18% to lower power output
  public static final double[][] kShooterVelocityTable = {
    {1.0, 1850},//2250
    {2.0, 2275},//2700
    {3.0, 2575},//3100
    {4.0, 2875},//3500
    {5.0, 3175},//3900
    {6.0, 3475}//4300
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

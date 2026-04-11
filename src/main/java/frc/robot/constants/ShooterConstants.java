package frc.robot.constants;

public class ShooterConstants {
  public static final int kFloorMotorId = 22;

  public static final int kRIndexerMotorId = 21;
  public static final int kLIndexerMotorId = 14;
  public static final int kMIndexerMotorId = 16;

  public static final int kRightShooterId = 18;
  public static final int kLeftShooterId = 17;
  public static final int kMiddleShooterId = 19;

  // BEST PRACTICE: Separate current limits allow independent tuning per motor type
  public static final int kMotorCurrentLimit = 60;  // Shooter wheels and indexers
  public static final int kFloorMotorCurrentLimit = 80;  // Higher limit for floor motor under load

  public static final double kShooterP = 0.00045; //0.00045
  public static final double kShooterI = 0.00000025; //0.00000025
  public static final double kShooterD = 0.001; //0.0001
  public static final double kShooterFF = 3.0 / MotorConstants.NeoVortex.kFreeSpeedRpm;

  // Indexer PID values
  public static final double kIndexerP = 0.00045;
  public static final double kIndexerI = 0.00000025;
  public static final double kIndexerD = 0.0001;
  public static final double kIndexerFF = 3.0 / MotorConstants.NeoVortex.kFreeSpeedRpm;

  // Floor motor PID values (for velocity control mode)
  public static final double kFloorMotorP = 0.00045;
  public static final double kFloorMotorI = 0.00000025;
  public static final double kFloorMotorD = 0.0001;
  public static final double kFloorMotorFF = 3.0 / MotorConstants.NeoVortex.kFreeSpeedRpm;

  // max rpm 6784+
  public static final double kShooterTargetRPM = 3000;
  public static final double kFloorMotorTargetRPM = 2500;  // Increased from 1000 to match ~50% duty cycle performance
  public static final double kIndexerMotorTargetRPM = 1000;

  // Pre-spin RPM cap (used when shooter active but not shooting)
  // Limits RPM to save energy until trigger is actually pulled
  public static final double kPreSpinRPMCap = 2000;

  // Shooter velocity lookup table: distance (meters) -> RPM
  // Reduced by ~18% to lower power output
  public static final double[][] kShooterVelocityTable = {
    // {1.75, 2950}, original values
    // {2.69, 3450},
    // {3.3, 4000},
    // {4.8, 4500},
    {1.82, 2900}, // 6 ft 2900 RPM
    {2.44, 3100}, // 8 ft 3100 RPM
    {2.86, 3150}, // 9.38 ft 3150 RMP
    {3.05, 3575}, // 10 ft 3575 RPM
    {3.66, 4100}, // 12 ft 4100 RPM
    {4.58, 4700}, // 15.03 ft 4700 RPM
  };

  // Velocity compensation for shooting while moving
  // Estimated average shot velocity in m/s (used to calculate flight time)
  public static final double kAverageShotVelocity = 10.0; // ~10 m/s for mid-range shots

  // Enable/disable velocity compensation (tune via NetworkTables)
  public static final boolean kVelocityCompensationEnabled = true;

  // RPM compensation factor: how much to adjust RPM per m/s of velocity
  // Positive value increases RPM when moving, compensating for effective distance change
  public static final double kRPMVelocityCompensationFactor = 0.0; // Start at 0, tune as needed

  // Angle compensation factor: multiplier for calculated angle offset
  // 1.0 = full compensation, 0.0 = no compensation
  public static final double kAngleCompensationFactor = 1.0;
}

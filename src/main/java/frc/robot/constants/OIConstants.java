package frc.robot.constants;

public final class OIConstants {
  public static final int kDriverControllerPort = 0;
  public static final int kMechanismControllerPort = 1;
  public static final double kDriveDeadband = 0.05;
  public static final double kRotationDeadband = 0.10;  // Higher deadband for rotation to prevent drift

  public static final double kNormalSpeedLimit = 0.5;
  public static final double kTurboSpeedLimit = 0.9;
  public static final double kPrecisionSpeedLimit = 0.25;
}

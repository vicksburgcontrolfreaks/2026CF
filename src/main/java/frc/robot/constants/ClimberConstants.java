package frc.robot.constants;

public class ClimberConstants {
  public static final int kClimberMotorId = 17;
  public static final int kMotorCurrentLimit = 40;

  public static final double kPositionP = 0.1;
  public static final double kPositionI = 0.0;
  public static final double kPositionD = 0.0;

  // Position setpoints (in rotations)
  public static final double kRetractedPosition = 0.0;
  public static final double kExtendedPosition = 100.0;

  public static final double kPositionTolerance = 2.0;

  public static final double kManualSpeed = 0.5;

  // Soft limits (in rotations)
  public static final double kSoftLimitMin = -5.0;
  public static final double kSoftLimitMax = 105.0;
}

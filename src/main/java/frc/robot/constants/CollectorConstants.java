package frc.robot.constants;

public class CollectorConstants {
  public static final int kUpperCollectorMotorId = 10;
  public static final int kLowerCollectorMotorId = 11;
  public static final int kHopperMotorId = 12;
  public static final int kLimitSwitchDIO = 0;

  public static final int kMotorCurrentLimit = 40;

  public static final double kCollectorSpeed = 0.2;
  public static final double kHopperSpeed = 0.3;

  // Spool unwound, pneumatic deployed
  public static final double kHopperPneumaticExtendedPosition = -208.0;
  // Spool wound, pneumatic pulled back
  public static final double kHopperPneumaticRetractedPosition = 0.0;
  public static final double kHopperPositionTolerance = 1;

  public static final double kHopperP = 0.1;
  public static final double kHopperI = 0.0;
  public static final double kHopperD = 0.0;
}

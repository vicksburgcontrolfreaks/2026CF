package frc.robot.constants;

public final class ModuleConstants {
  // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
  public static final int kDrivingMotorPinionTeeth = 14;

  public static final double kDrivingMotorFreeSpeedRps = MotorConstants.NeoVortex.kFreeSpeedRpm / 60;
  public static final double kWheelDiameterMeters = 0.0762;
  public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
  public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
  public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;
}

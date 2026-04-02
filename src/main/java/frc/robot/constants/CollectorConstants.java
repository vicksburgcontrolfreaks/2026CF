package frc.robot.constants;

public class CollectorConstants {
  public static final int kUpperCollectorMotorId = 10;
  // Lower collector motor removed - CAN ID 11 no longer in use
  public static final int kHopperMotorId = 12;

  public static final int kMotorCurrentLimit = 60;

  public static final double kCollectorSpeed = 0.35;  // Duty cycle (deprecated, use RPM)
  public static final double kCollectorTargetRPM = 2014.0;  // Target velocity in RPM (measured with tachometer at 35% duty cycle)

  // Collector velocity PID constants (for RPM control)
  // Note: At P=0.0001, target 3000 RPM only reached 1107 RPM - P was too low
  // Measured: 35% duty cycle = 2014 RPM → kFF = 0.35/2014 = 0.000174
  public static final double kCollectorP = 0.0002;  // Proportional gain for velocity error correction
  public static final double kCollectorI = 0.0;     // Not needed for velocity control
  public static final double kCollectorD = 0.0;     // Not needed for velocity control
  public static final double kCollectorFF = 0.000174; // Feedforward: duty cycle per RPM (measured)

  public static final double kUpPosition = 0.01;
  public static final double kHalfwayPosition = 0.095;  // Halfway between up and down
  public static final double kDownPosition = 0.180;

  public static final double kHopperP = 2.5;
  public static final double kHopperI = 0.0;
  public static final double kHopperD = 0.0;

  // Hopper MAXMotion limits
  public static final double kHopperMaxVelocity = 80.0;     // mechanism RPM (60:1 gearing)
  public static final double kHopperMaxAcceleration = 160.0; // mechanism RPM/s
}

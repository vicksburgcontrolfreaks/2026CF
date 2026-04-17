package frc.robot.constants;

public class CollectorConstants {
  public static final int kUpperCollectorMotorId = 10;
  // Lower collector motor removed - CAN ID 11 no longer in use
  public static final int kHopperMotorId = 12;

  public static final int kMotorCurrentLimit = 80;

  public static final double kCollectorTargetRPM = 2250.0;  // Target velocity in RPM (tuned for reliable collection)

  // Collector velocity PID constants (for RPM control)
  // Tuned values achieve 2500 RPM with reliable collection performance
  public static final double kCollectorP = 0.0002;  // Proportional gain for velocity error correction
  public static final double kCollectorI = 0.0;     // Not needed for velocity control
  public static final double kCollectorD = 0.0;     // Not needed for velocity control
  public static final double kCollectorFF = 0.003;  // Feedforward: duty cycle per RPM (tuned)

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

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public final class AutoConstants {
  public static final double kMaxSpeedMetersPerSecond = 3.5;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

  public static final double kPXController = 1;
  public static final double kPYController = 1;
  public static final double kPThetaController = 1;

  public static final double kPTranslation = 5.0;
  public static final double kITranslation = 0.0;
  public static final double kDTranslation = 0.0;

  public static final double kPRotation = 15.0;
  public static final double kIRotation = 0.25;
  public static final double kDRotation = 0.1;

  public static final Translation2d blueTarget = new Translation2d(4.639, 4.02);
  public static final Translation2d redTarget = new Translation2d(11.942, 4.02);

  // Rotational PID for waypoint navigation and auto-aim
  // Reverted to main branch values to fix unwanted rotation during teleop
  public static final double kRotateToTargetP = 0.015;
  public static final double kRotateToTargetI = 0.0;
  public static final double kRotateToTargetD = 0.0;
  public static final double kRotateToTargetTolerance = 3.0;
  public static final double kRotateToTargetMaxVelocity = 0.5;

  // Waypoint positional PID — controls drive speed as a function of distance to waypoint
  public static final double kWaypointP        = 1.15;  // proportional: speed per meter of error
  public static final double kWaypointI        = 0.0;
  public static final double kWaypointD        = 0.001;
  public static final double kWaypointMaxSpeed = 0.5;   // fraction of kMaxSpeedMetersPerSecond
}

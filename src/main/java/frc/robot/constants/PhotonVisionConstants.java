package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class PhotonVisionConstants {
  public static final String kCameraFrontName = "camera-front";
  public static final String kCameraBackName = "camera-back";
  public static final String kCameraLeftName = "camera-left";
  public static final String kCameraRightName = "camera-right";

  // Robot-to-Camera Transform3d(Translation3d, Rotation3d):
  // Translation3d(X, Y, Z):
  //   X: forward(+)/back(-) from robot center (in meters)
  //   Y: left(+)/right(-) from robot center
  //   Z: up(+)/down(-) from robot center
  // Rotation3d(roll, pitch, yaw):
  //   roll: rotation around X-axis (tilt left/right) in radians
  //   pitch: rotation around Y-axis (tilt up/down) in radians
  //   yaw: rotation around Z-axis (spin left/right) in radians - 0° = forward, 90° = left, 180° = back, -90° = right

  public static final Transform3d kRobotToFrontCamera = new Transform3d(
    new Translation3d(Units.inchesToMeters(-5.771), Units.inchesToMeters(0), Units.inchesToMeters(28.15)),
    new Rotation3d(0, Math.toRadians(6), Math.toRadians(0)) // roll: 0°, pitch: 6° (tilted up), yaw: 0° (forward)
  );

  public static final Transform3d kRobotToBackCamera = new Transform3d(
    new Translation3d(Units.inchesToMeters(-8.845), Units.inchesToMeters(7.60), Units.inchesToMeters(26.566)),
    new Rotation3d(0, Math.toRadians(6), Math.toRadians(180)) // roll: 0°, pitch: 6° (tilted up), yaw: 180° (backward)
  );

  public static final Transform3d kRobotToLeftCamera = new Transform3d(
    new Translation3d(Units.inchesToMeters(2.89), Units.inchesToMeters(13.75), Units.inchesToMeters(24.61)),
    new Rotation3d(0, Math.toRadians(0), Math.toRadians(90)) // roll: 0°, pitch: 0°, yaw: 90° (left)
  );

  public static final Transform3d kRobotToRightCamera = new Transform3d(
    new Translation3d(Units.inchesToMeters(2.89), Units.inchesToMeters(-13.75), Units.inchesToMeters(24.61)),
    new Rotation3d(0, Math.toRadians(0), Math.toRadians(-90)) // roll: 0°, pitch: 0°, yaw: -90° (right)
  );

  public static final double[] kSingleTagStdDevs = {1.5, 1.5, 3.0};
  public static final double[] kMultiTagStdDevs = {0.5, 0.5, 1.0};

  public static final double kMaxAmbiguity = 0.2;
  public static final double kMaxTagDistance = 5.0;
  public static final int kMinTagsForHighConfidence = 2;

  public static final double kDistanceFactorThreshold = 2.0;
  public static final double kConfidenceTagCountMultiplier = 10.0;
  public static final double kConfidenceAmbiguityMultiplier = 5.0;
}

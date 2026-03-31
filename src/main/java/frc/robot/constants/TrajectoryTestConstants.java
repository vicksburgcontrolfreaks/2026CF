package frc.robot.constants;

/**
 * Constants for trajectory testing.
 *
 * Hardware adjustment: Shooter trajectory can be adjusted in 2-degree increments.
 * Baseline is 22 degrees.
 *
 * Test objectives:
 * 1. Scoring from extended distances
 * 2. Clearing balls to our side of field when hub inactive
 */
public class TrajectoryTestConstants {
  // Physical trajectory angle (hardware adjustment)
  public static final double kBaselineTrajectoryAngle = 22.0; // degrees
  public static final double kTrajectoryIncrementSize = 2.0; // degrees per adjustment

  // Test mode RPM range
  public static final double kTestRPMMin = 2000.0;
  public static final double kTestRPMMax = 5000.0;
  public static final double kTestRPMDefault = 3500.0;

  // Test distances for scoring (meters)
  public static final double[] kTestDistances = {
    1.75,  // Close
    2.69,  // Mid
    3.3,   // Far (current max)
    4.0,   // Extended 1
    4.5,   // Extended 2
    5.0    // Extended 3
  };

  // Clearing shot parameters
  // When hub is inactive, launch balls back to our side from neutral/opponent zones
  public static final double kClearingShotRPM = 4500.0; // High arc, long distance
  public static final String kClearingShotNotes = "High trajectory for clearing balls across field";
}

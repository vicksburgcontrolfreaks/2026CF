package frc.robot.constants;

public class LEDConstants {
  // Hardware configuration
  public static final int kLEDPort = 0;  // PWM port on RoboRIO
  public static final int kLEDCount = 36;  // 36 addressable RGB LEDs

  // LED segment divisions for auto selection display
  public static final int kLeftSegmentStart = 0;
  public static final int kLeftSegmentEnd = 11;  // First 12 LEDs (0-11)
  public static final int kCenterSegmentStart = 12;
  public static final int kCenterSegmentEnd = 23;  // Middle 12 LEDs (12-23)
  public static final int kRightSegmentStart = 24;
  public static final int kRightSegmentEnd = 35;  // Last 12 LEDs (24-35)

  // Color definitions (RGB values 0-255)
  // Note: Red and Green channels are swapped on this LED strip
  public static final int[] kRedColor = {0, 255, 0};  // Green channel for red
  public static final int[] kBlueColor = {0, 0, 255};
  public static final int[] kGreenColor = {255, 0, 0};  // Red channel for green
  public static final int[] kYellowColor = {0, 255, 255};  // Green + Blue for yellow
  public static final int[] kOffColor = {0, 0, 0};

  // Hub timing constants (match time counting DOWN from 135)
  // Teleop starts at 135 seconds remaining
  public static final double kTransitionShiftEnd = 130.0;  // Hub always active before this
  public static final double kShift1End = 105.0;
  public static final double kShift2End = 80.0;
  public static final double kShift3End = 55.0;
  public static final double kShift4End = 30.0;
  public static final double kEndgameStart = 30.0;  // Hub always active after this

  // Warning times - flash LEDs this many seconds before hub state changes
  public static final double kHubWarningTime = 3.0;  // Flash 3 seconds before change

  // Flash timing for animations
  public static final double kFlashPeriod = 0.5;  // Flash every 0.5 seconds (2 Hz)
}

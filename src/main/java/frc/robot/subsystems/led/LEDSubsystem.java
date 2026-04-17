// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

/**
 * LED subsystem for controlling 36 addressable RGB LEDs on a 12V strip.
 * Displays status information for auto selection, AprilTag detection, hub timing, and match state.
 */
public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  // LED mode state
  private LEDMode m_currentMode = LEDMode.DISABLED;
  private AutoPosition m_autoPosition = AutoPosition.CENTER;

  // Flash timing
  private double m_lastFlashTime = 0;
  private boolean m_flashState = false;

  // Hub timing state
  private char m_hubGameData = 'U';  // 'R', 'B', or 'U' (unknown)

  /**
   * LED display modes
   */
  public enum LEDMode {
    DISABLED,           // Robot disabled - show auto selection
    AUTONOMOUS,         // Autonomous mode - flash alliance color
    TELEOP,             // Teleop mode - show hub status
    NO_APRILTAGS        // No AprilTags detected - flash yellow
  }

  /**
   * Auto position selection
   */
  public enum AutoPosition {
    LEFT,
    CENTER,
    RIGHT
  }

  /**
   * Creates a new LEDSubsystem.
   */
  public LEDSubsystem() {
    m_led = new AddressableLED(LEDConstants.kLEDPort);
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDCount);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();

    // Initialize with all LEDs off
    clearAllLEDs();
  }

  @Override
  public void periodic() {
    // Update flash state based on timer
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - m_lastFlashTime >= LEDConstants.kFlashPeriod) {
      m_flashState = !m_flashState;
      m_lastFlashTime = currentTime;
    }

    // Update LED pattern based on current mode
    switch (m_currentMode) {
      case DISABLED:
        updateDisabledPattern();
        break;
      case AUTONOMOUS:
        updateAutonomousPattern();
        break;
      case TELEOP:
        updateTeleopPattern();
        break;
      case NO_APRILTAGS:
        updateNoAprilTagsPattern();
        break;
    }

    // Update LED strip with current buffer state
    m_led.setData(m_ledBuffer);
  }

  /**
   * Sets the LED mode (disabled, autonomous, teleop, etc.)
   */
  public void setMode(LEDMode mode) {
    m_currentMode = mode;
  }

  /**
   * Sets the auto position for displaying auto selection pattern
   */
  public void setAutoPosition(AutoPosition position) {
    m_autoPosition = position;
  }

  /**
   * Sets the auto position based on robot pose (y coordinate)
   * Red: y < 3.0 = LEFT, y > 5.0 = RIGHT, else CENTER
   * Blue: y > 5.0 = LEFT, y < 3.0 = RIGHT, else CENTER
   */
  public void setAutoPositionFromPose(double x, double y) {
    boolean isRed = x > 8.27;

    if (isRed) {
      if (y < 3.0) {
        m_autoPosition = AutoPosition.LEFT;
      } else if (y > 5.0) {
        m_autoPosition = AutoPosition.RIGHT;
      } else {
        m_autoPosition = AutoPosition.CENTER;
      }
    } else {
      if (y > 5.0) {
        m_autoPosition = AutoPosition.LEFT;
      } else if (y < 3.0) {
        m_autoPosition = AutoPosition.RIGHT;
      } else {
        m_autoPosition = AutoPosition.CENTER;
      }
    }
  }

  /**
   * Sets the hub game data ('R' or 'B' indicating which alliance goes inactive first)
   */
  public void setHubGameData(char gameData) {
    m_hubGameData = gameData;
  }

  /**
   * Updates the disabled pattern - shows auto selection with alliance color
   */
  private void updateDisabledPattern() {
    clearAllLEDs();

    // Get alliance color
    int[] allianceColor = getAllianceColor();

    // Set LEDs based on auto position
    switch (m_autoPosition) {
      case LEFT:
        // Left segment = alliance color, rest = green
        setSegment(LEDConstants.kLeftSegmentStart, LEDConstants.kLeftSegmentEnd, allianceColor);
        setSegment(LEDConstants.kCenterSegmentStart, LEDConstants.kRightSegmentEnd, LEDConstants.kGreenColor);
        break;

      case CENTER:
        // Center segment = alliance color, rest = green
        setSegment(LEDConstants.kLeftSegmentStart, LEDConstants.kLeftSegmentEnd, LEDConstants.kGreenColor);
        setSegment(LEDConstants.kCenterSegmentStart, LEDConstants.kCenterSegmentEnd, allianceColor);
        setSegment(LEDConstants.kRightSegmentStart, LEDConstants.kRightSegmentEnd, LEDConstants.kGreenColor);
        break;

      case RIGHT:
        // Right segment = alliance color, rest = green
        setSegment(LEDConstants.kLeftSegmentStart, LEDConstants.kCenterSegmentEnd, LEDConstants.kGreenColor);
        setSegment(LEDConstants.kRightSegmentStart, LEDConstants.kRightSegmentEnd, allianceColor);
        break;
    }
  }

  /**
   * Updates the autonomous pattern - flash alliance color
   */
  private void updateAutonomousPattern() {
    int[] color = m_flashState ? getAllianceColor() : LEDConstants.kOffColor;
    setAllLEDs(color);
  }

  /**
   * Updates the teleop pattern - show hub status and warnings
   */
  private void updateTeleopPattern() {
    double matchTime = DriverStation.getMatchTime();

    // Check if we're approaching a hub state change
    boolean approachingChange = isApproachingHubChange(matchTime);

    if (approachingChange) {
      // Flash to warn of upcoming hub state change
      int[] color = m_flashState ? getAllianceColor() : LEDConstants.kOffColor;
      setAllLEDs(color);
    } else {
      // Show hub status:
      // - Our hub active = our alliance color
      // - Our hub inactive + opponent hub active = opponent alliance color
      // - Both hubs inactive = off
      boolean ourHubActive = isHubActive(matchTime);
      boolean opponentHubActive = isOpponentHubActive(matchTime);

      int[] color;
      if (ourHubActive) {
        // Our hub is active - show our alliance color
        color = getAllianceColor();
      } else if (opponentHubActive) {
        // Our hub inactive, opponent hub active - show opponent alliance color
        color = getOpponentAllianceColor();
      } else {
        // Both hubs inactive - turn off
        color = LEDConstants.kOffColor;
      }

      setAllLEDs(color);
    }
  }

  /**
   * Updates the no AprilTags pattern - flash yellow
   */
  private void updateNoAprilTagsPattern() {
    int[] color = m_flashState ? LEDConstants.kYellowColor : LEDConstants.kOffColor;
    setAllLEDs(color);
  }

  /**
   * Determines if the hub is currently active based on match time and game data
   * Match time counts DOWN from 135 seconds
   */
  private boolean isHubActive(double matchTime) {
    // Always active during endgame (< 30 seconds remaining)
    if (matchTime <= LEDConstants.kEndgameStart) {
      return true;
    }

    // Always active during transition shift (> 130 seconds remaining)
    if (matchTime > LEDConstants.kTransitionShiftEnd) {
      return true;
    }

    // Determine which alliance goes inactive first (scored more fuel in auto)
    boolean weGoInactiveFirst = false;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red && m_hubGameData == 'R') {
        weGoInactiveFirst = true;
      } else if (alliance.get() == Alliance.Blue && m_hubGameData == 'B') {
        weGoInactiveFirst = true;
      }
    }

    // Determine current shift and hub status
    // Alliance that goes inactive first: Active in Shifts 2 and 4
    // Other alliance: Active in Shifts 1 and 3
    if (matchTime > LEDConstants.kShift1End) {
      // Shift 1 (105-130 seconds)
      return !weGoInactiveFirst;
    } else if (matchTime > LEDConstants.kShift2End) {
      // Shift 2 (80-105 seconds)
      return weGoInactiveFirst;
    } else if (matchTime > LEDConstants.kShift3End) {
      // Shift 3 (55-80 seconds)
      return !weGoInactiveFirst;
    } else if (matchTime > LEDConstants.kShift4End) {
      // Shift 4 (30-55 seconds)
      return weGoInactiveFirst;
    }

    // Default to active if time is unknown
    return true;
  }

  /**
   * Determines if the opponent's hub is currently active based on match time and game data
   * Inverse of our hub status during normal shifts
   */
  private boolean isOpponentHubActive(double matchTime) {
    // Always active during endgame (< 30 seconds remaining)
    if (matchTime <= LEDConstants.kEndgameStart) {
      return true;
    }

    // Always active during transition shift (> 130 seconds remaining)
    if (matchTime > LEDConstants.kTransitionShiftEnd) {
      return true;
    }

    // Determine which alliance goes inactive first (scored more fuel in auto)
    boolean weGoInactiveFirst = false;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red && m_hubGameData == 'R') {
        weGoInactiveFirst = true;
      } else if (alliance.get() == Alliance.Blue && m_hubGameData == 'B') {
        weGoInactiveFirst = true;
      }
    }

    // Determine current shift and opponent hub status
    // Opponent is inverse of our status during shifts
    // Alliance that goes inactive first: Active in Shifts 2 and 4
    // Other alliance: Active in Shifts 1 and 3
    if (matchTime > LEDConstants.kShift1End) {
      // Shift 1 (105-130 seconds)
      return weGoInactiveFirst;  // Opposite of our hub
    } else if (matchTime > LEDConstants.kShift2End) {
      // Shift 2 (80-105 seconds)
      return !weGoInactiveFirst;  // Opposite of our hub
    } else if (matchTime > LEDConstants.kShift3End) {
      // Shift 3 (55-80 seconds)
      return weGoInactiveFirst;  // Opposite of our hub
    } else if (matchTime > LEDConstants.kShift4End) {
      // Shift 4 (30-55 seconds)
      return !weGoInactiveFirst;  // Opposite of our hub
    }

    // Default to active if time is unknown
    return true;
  }

  /**
   * Checks if we're approaching a hub state change (within warning time)
   */
  private boolean isApproachingHubChange(double matchTime) {
    // Check if we're within warning time of any shift boundary
    double[] shiftBoundaries = {
      LEDConstants.kTransitionShiftEnd,
      LEDConstants.kShift1End,
      LEDConstants.kShift2End,
      LEDConstants.kShift3End,
      LEDConstants.kShift4End,
      LEDConstants.kEndgameStart
    };

    for (double boundary : shiftBoundaries) {
      double timeToBoundary = Math.abs(matchTime - boundary);
      if (timeToBoundary <= LEDConstants.kHubWarningTime) {
        return true;
      }
    }

    return false;
  }

  /**
   * Gets the alliance color (red or blue)
   */
  private int[] getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return LEDConstants.kRedColor;
    } else {
      return LEDConstants.kBlueColor;
    }
  }

  /**
   * Gets the opponent alliance color (opposite of ours)
   */
  private int[] getOpponentAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      // We're red, opponent is blue
      return LEDConstants.kBlueColor;
    } else {
      // We're blue, opponent is red
      return LEDConstants.kRedColor;
    }
  }

  /**
   * Sets all LEDs to a specific color
   */
  private void setAllLEDs(int[] color) {
    for (int i = 0; i < LEDConstants.kLEDCount; i++) {
      setLED(i, color);
    }
  }

  /**
   * Sets a segment of LEDs to a specific color
   */
  private void setSegment(int start, int end, int[] color) {
    for (int i = start; i <= end && i < LEDConstants.kLEDCount; i++) {
      setLED(i, color);
    }
  }

  /**
   * Sets a specific LED to a color.
   *
   * @param index LED index (0-based)
   * @param color RGB color array [r, g, b] with values 0-255
   */
  private void setLED(int index, int[] color) {
    if (index >= 0 && index < LEDConstants.kLEDCount && color.length == 3) {
      m_ledBuffer.setRGB(index, color[0], color[1], color[2]);
    }
  }

  /**
   * Turns off all LEDs.
   */
  public void clearAllLEDs() {
    for (int i = 0; i < LEDConstants.kLEDCount; i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import java.util.List;

/**
 * Subsystem for controlling a strip of 23 addressable LEDs (WS2812B/NeoPixel).
 * Provides methods for solid colors, animations, and effects.
 */
public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  // Animation state
  private LEDPattern m_currentPattern;
  private int[] m_currentColor;
  private double m_animationStartTime;
  private int m_rainbowFirstPixelHue;

  // Individual LED state tracking
  private boolean m_manualMode;
  private boolean m_led2to5State; // Tracks LEDs 2-5 toggle state
  private int m_led8to10Counter; // Tracks which LEDs 8-10 are on (0-3)

  // Binding set selection (true = set 1, false = set 2)
  private boolean m_useBindingSet1;

  /**
   * Enum for different LED patterns
   */
  public enum LEDPattern {
    OFF,
    SOLID,
    BLINK,
    BREATHE,
    RAINBOW,
    CHASE
  }

  /**
   * Creates a new LEDSubsystem.
   */
  public LEDSubsystem() {
    try {
      m_led = new AddressableLED(LEDConstants.kLEDPort);
      m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDCount);

      // Set the LED strip length
      m_led.setLength(m_ledBuffer.getLength());

      // WS2811 (12V) LEDs use the same timing as WS2812B
      // The roboRIO AddressableLED should work with WS2811
      // IMPORTANT: WS2811 requires external 12V power supply!

      // Set bit timing for WS2811 compatibility (optional - usually not needed)
      // m_led.setBitTiming(400, 850, 450, 600); // WS2811 timing in nanoseconds

      // Start the LED output BEFORE setting data
      m_led.start();

      // Initialize with LEDs off
      m_currentPattern = LEDPattern.OFF;
      m_currentColor = LEDConstants.kOffColor;
      m_animationStartTime = Timer.getFPGATimestamp();
      m_rainbowFirstPixelHue = 0;
      m_manualMode = false;
      m_led2to5State = false;
      m_led8to10Counter = 0;
      m_useBindingSet1 = true; // Start with binding set 1

      // Set all LEDs to bright white for 0.5 seconds as a startup test
      // This helps verify the LEDs are connected and working
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, LEDConstants.kStartupTestR, LEDConstants.kStartupTestG, LEDConstants.kStartupTestB);
      }
      m_led.setData(m_ledBuffer);

      // Log initialization
      DriverStation.reportWarning("LED Subsystem initialized on PWM port " + LEDConstants.kLEDPort + " with " + LEDConstants.kLEDCount + " LEDs - Watch for white flash!", false);
    } catch (Exception e) {
      DriverStation.reportError("LED Subsystem failed to initialize: " + e.getMessage(), e.getStackTrace());
      throw e;
    }
  }

  @Override
  public void periodic() {
    // Skip automatic pattern updates if in manual mode
    if (!m_manualMode) {
      // Update LED pattern based on current mode
      switch (m_currentPattern) {
        case OFF:
          setAllLEDs(0, 0, 0);
          break;
        case SOLID:
          setAllLEDs(m_currentColor[0], m_currentColor[1], m_currentColor[2]);
          break;
        case BLINK:
          updateBlink();
          break;
        case BREATHE:
          updateBreathe();
          break;
        case RAINBOW:
          updateRainbow();
          break;
        case CHASE:
          updateChase();
          break;
      }
    }

    // Send the buffer to the LED strip
    m_led.setData(m_ledBuffer);
  }

  /**
   * Sets all LEDs to a solid color
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  public void setSolidColor(int r, int g, int b) {
    m_currentPattern = LEDPattern.SOLID;
    m_currentColor = new int[]{r, g, b};
  }

  /**
   * Sets all LEDs to a solid color using an RGB array
   * @param rgb RGB array [r, g, b] with values 0-255
   */
  public void setSolidColor(int[] rgb) {
    setSolidColor(rgb[0], rgb[1], rgb[2]);
  }

  /**
   * Sets all LEDs to blink a specific color
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  public void setBlink(int r, int g, int b) {
    m_currentPattern = LEDPattern.BLINK;
    m_currentColor = new int[]{r, g, b};
    m_animationStartTime = Timer.getFPGATimestamp();
  }

  /**
   * Sets all LEDs to blink a specific color using an RGB array
   * @param rgb RGB array [r, g, b] with values 0-255
   */
  public void setBlink(int[] rgb) {
    setBlink(rgb[0], rgb[1], rgb[2]);
  }

  /**
   * Sets all LEDs to breathe (fade in/out) with a specific color
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  public void setBreathe(int r, int g, int b) {
    m_currentPattern = LEDPattern.BREATHE;
    m_currentColor = new int[]{r, g, b};
    m_animationStartTime = Timer.getFPGATimestamp();
  }

  /**
   * Sets all LEDs to breathe (fade in/out) with a specific color using an RGB array
   * @param rgb RGB array [r, g, b] with values 0-255
   */
  public void setBreathe(int[] rgb) {
    setBreathe(rgb[0], rgb[1], rgb[2]);
  }

  /**
   * Sets all LEDs to display a rainbow pattern
   */
  public void setRainbow() {
    m_currentPattern = LEDPattern.RAINBOW;
    m_rainbowFirstPixelHue = 0;
  }

  /**
   * Sets all LEDs to display a chase pattern
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  public void setChase(int r, int g, int b) {
    m_currentPattern = LEDPattern.CHASE;
    m_currentColor = new int[]{r, g, b};
    m_animationStartTime = Timer.getFPGATimestamp();
  }

  /**
   * Sets all LEDs to display a chase pattern using an RGB array
   * @param rgb RGB array [r, g, b] with values 0-255
   */
  public void setChase(int[] rgb) {
    setChase(rgb[0], rgb[1], rgb[2]);
  }

  /**
   * Turns off all LEDs
   */
  public void turnOff() {
    m_currentPattern = LEDPattern.OFF;
    m_currentColor = LEDConstants.kOffColor;
  }

  /**
   * Sets a specific LED to a color and immediately updates the strip
   * IMPORTANT: AndyMark 12V LED strips use GRB color order, not RGB!
   * @param index LED index (0-22)
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  public void setLED(int index, int r, int g, int b) {
    if (index >= 0 && index < LEDConstants.kLEDCount) {
      // AndyMark LEDs use GRB order instead of RGB
      // So we need to swap red and green when calling setRGB
      m_ledBuffer.setRGB(index, g, r, b);  // NOTE: Green and Red are swapped!
      // Immediately send data to LED strip when in manual mode
      if (m_manualMode) {
        m_led.setData(m_ledBuffer);
      }
    }
  }

  /**
   * Sets all LEDs to a specific color
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  private void setAllLEDs(int r, int g, int b) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    // Data will be sent in periodic() or immediately if in manual mode
  }

  /**
   * Updates the blink animation
   */
  private void updateBlink() {
    double currentTime = Timer.getFPGATimestamp();
    double elapsed = (currentTime - m_animationStartTime) * 1000; // Convert to milliseconds

    boolean isOn = ((int)(elapsed / LEDConstants.kBlinkSpeed) % 2) == 0;

    if (isOn) {
      setAllLEDs(m_currentColor[0], m_currentColor[1], m_currentColor[2]);
    } else {
      setAllLEDs(0, 0, 0);
    }
  }

  /**
   * Updates the breathe animation (smooth fade in/out)
   */
  private void updateBreathe() {
    double currentTime = Timer.getFPGATimestamp();
    double elapsed = (currentTime - m_animationStartTime) * 1000; // Convert to milliseconds

    // Create a sine wave for smooth breathing effect
    double breatheCycle = (elapsed / LEDConstants.kBreatheSpeed) % (2 * Math.PI);
    double brightness = (Math.sin(breatheCycle) + 1.0) / 2.0; // 0.0 to 1.0

    int r = (int)(m_currentColor[0] * brightness);
    int g = (int)(m_currentColor[1] * brightness);
    int b = (int)(m_currentColor[2] * brightness);

    setAllLEDs(r, g, b);
  }

  /**
   * Updates the rainbow animation
   */
  private void updateRainbow() {
    // For every pixel
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (m_rainbowFirstPixelHue + (i * LEDConstants.kRainbowHueModulo / m_ledBuffer.getLength())) % LEDConstants.kRainbowHueModulo;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += LEDConstants.kRainbowHueIncrement;
    // Check bounds
    m_rainbowFirstPixelHue %= LEDConstants.kRainbowHueModulo;
  }

  /**
   * Updates the chase animation (moving dot)
   */
  private void updateChase() {
    double currentTime = Timer.getFPGATimestamp();
    double elapsed = (currentTime - m_animationStartTime) * 1000; // Convert to milliseconds

    // Calculate which LED should be lit
    int position = (int)((elapsed / LEDConstants.kBlinkSpeed) % m_ledBuffer.getLength());

    // Turn off all LEDs
    setAllLEDs(0, 0, 0);

    // Light up the current position and a few around it for a tail effect
    for (int i = 0; i < LEDConstants.kChaseTailLength; i++) {
      int ledIndex = (position - i + m_ledBuffer.getLength()) % m_ledBuffer.getLength();
      double brightness = 1.0 - (i * LEDConstants.kChaseBrightnessFade); // Fade the tail
      int r = (int)(m_currentColor[0] * brightness);
      int g = (int)(m_currentColor[1] * brightness);
      int b = (int)(m_currentColor[2] * brightness);
      setLED(ledIndex, r, g, b);
    }
  }

  /**
   * Gets the current LED pattern
   * @return Current LED pattern
   */
  public LEDPattern getCurrentPattern() {
    return m_currentPattern;
  }

  /**
   * Gets the current color
   * @return Current RGB color array
   */
  public int[] getCurrentColor() {
    return m_currentColor;
  }

  /**
   * Enables manual mode WITHOUT clearing LEDs (allows multiple controls)
   */
  public void enableManualMode() {
    if (!m_manualMode) {
      m_manualMode = true;
      // Don't clear LEDs - allows multiple controls to work together
    }
  }

  /**
   * Disables manual mode and returns to pattern control
   */
  public void disableManualMode() {
    m_manualMode = false;
  }

  // ========== BINDING SET 1 METHODS ==========

  /**
   * Sets LED 1 (index 0) to green
   */
  public void setLED1Green() {
    System.out.println(">>> A BUTTON PRESSED - Setting LED 1 to green <<<");
    DriverStation.reportWarning(">>> A BUTTON PRESSED - Setting LED 1 to green <<<", false);
    enableManualMode();
    setLED(0, 0, 255, 0);
  }

  /**
   * Clears LED 1 (index 0)
   */
  public void clearLED1() {
    setLED(0, 0, 0, 0);
  }

  /**
   * Toggles LEDs 2-5 (indices 1-4) between blue and off
   */
  public void toggleLED2to5Blue() {
    System.out.println(">>> B BUTTON PRESSED - Toggling LEDs 2-5 <<<");
    enableManualMode();
    m_led2to5State = !m_led2to5State;
    if (m_led2to5State) {
      for (int i = 1; i <= 4; i++) {
        setLED(i, 0, 0, 255);
      }
    } else {
      for (int i = 1; i <= 4; i++) {
        setLED(i, 0, 0, 0);
      }
    }
  }

  /**
   * Cycles through LEDs 8-10 (indices 7-9) one at a time
   * Each press turns on the next LED in sequence
   */
  public void cycleLED8to10() {
    enableManualMode();

    // Increment counter (wraps from 3 back to 0)
    m_led8to10Counter = (m_led8to10Counter + 1) % LEDConstants.kCycleCounterModulo;

    // Turn off all LEDs 8-10 first
    for (int i = 7; i <= 9; i++) {
      setLED(i, 0, 0, 0);
    }

    // Turn on LEDs based on counter
    if (m_led8to10Counter > 0) {
      for (int i = 0; i < m_led8to10Counter; i++) {
        setLED(7 + i, 0, 255, 0); // Green
      }
    }
  }

  // ========== BINDING SET 2 METHODS (Currently blank/inactive) ==========

  /**
   * Placeholder for binding set 2 - A button action
   */
  public void bindingSet2_A() {
    // Currently inactive - add functionality when needed
  }

  /**
   * Placeholder for binding set 2 - A button release action
   */
  public void bindingSet2_A_Release() {
    // Currently inactive - add functionality when needed
  }// comment-(P.Slone, 2026);

  /**
   * Placeholder for binding set 2 - B button action
   */
  public void bindingSet2_B() {
    // Currently inactive - add functionality when needed
  }

  /**
   * Placeholder for binding set 2 - D-pad up action
   */
  public void bindingSet2_DpadUp() {
    // Currently inactive - add functionality when needed
  }


  /**
   * Turns off all LEDs and resets all state variables
   */
  public void turnOffAllAndReset() {
    System.out.println(">>> Y BUTTON PRESSED - Turning off all LEDs and resetting state <<<");
    enableManualMode();

    // Reset all state variables
    m_led2to5State = false;
    m_led8to10Counter = 0;

    // Turn off all LEDs
    for (int i = 0; i < LEDConstants.kLEDCount; i++) {
      setLED(i, 0, 0, 0);
    }
  }

  /**
   * Toggles between binding set 1 and binding set 2
   */
  public void toggleBindingSet() {
    m_useBindingSet1 = !m_useBindingSet1;
    System.out.println(">>> Switched to binding set " + (m_useBindingSet1 ? "1" : "2") + " <<<");
    DriverStation.reportWarning("Switched to LED binding set " + (m_useBindingSet1 ? "1" : "2"), false);
  }

  /**
   * Gets the current binding set (true = set 1, false = set 2)
   */
  public boolean isBindingSet1() {
    return m_useBindingSet1;
  }


  /**
   * Sets all LEDs to red (for disabled/idle mode)
   */
  public void setAllLEDsRed() {
    disableManualMode(); // Exit manual mode
    m_currentPattern = LEDPattern.SOLID;
    m_currentColor = LEDConstants.kRedColor;
    // Immediately update to show red
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Remember: GRB color order, so red (255,0,0) becomes (0,255,0) in buffer
      m_ledBuffer.setRGB(i, 0, 255, 0); // Green channel = Red in GRB
    }
    m_led.setData(m_ledBuffer);
  }

  /**
   * Sets LEDs based on detected AprilTag IDs.
   * Each LED number corresponds to an AprilTag ID (e.g., LED 1 = Tag 1, LED 2 = Tag 2, etc.)
   *
   * @param tagIDs List of detected AprilTag IDs
   * @param color RGB color array [r, g, b] to use for detected tags
   */
  public void setAprilTagLEDs(List<Integer> tagIDs, int[] color) {
    enableManualMode();

    // First, turn off all LEDs
    for (int i = 0; i < LEDConstants.kLEDCount; i++) {
      setLED(i, 0, 0, 0);
    }

    // Light up LEDs corresponding to detected tag IDs
    for (Integer tagID : tagIDs) {
      // Check if tag ID is within LED range (1-23 for 23 LEDs)
      if (tagID >= 1 && tagID <= LEDConstants.kLEDCount) {
        // LED index is tagID - 1 (since LEDs are 0-indexed)
        setLED(tagID - 1, color[0], color[1], color[2]);
      }
    }
  }

  /**
   * Sets LEDs based on detected AprilTag IDs using individual RGB values.
   *
   * @param tagIDs List of detected AprilTag IDs
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  public void setAprilTagLEDs(List<Integer> tagIDs, int r, int g, int b) {
    setAprilTagLEDs(tagIDs, new int[]{r, g, b});
  }

  /**
   * Clears all AprilTag indicator LEDs
   */
  public void clearAprilTagLEDs() {
    enableManualMode();
    for (int i = 0; i < LEDConstants.kLEDCount; i++) {
      setLED(i, 0, 0, 0);
    }
  }
}

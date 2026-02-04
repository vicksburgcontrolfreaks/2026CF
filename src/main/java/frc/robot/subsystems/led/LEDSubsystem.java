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

      m_led.setLength(m_ledBuffer.getLength());

      // IMPORTANT: WS2811 requires external 12V power supply!
      // Set bit timing for WS2811 compatibility (optional - usually not needed)
      // m_led.setBitTiming(400, 850, 450, 600); // WS2811 timing in nanoseconds

      m_led.start();

      // Initialize with LEDs off
      m_currentPattern = LEDPattern.OFF;
      m_currentColor = LEDConstants.kOffColor;
      m_animationStartTime = Timer.getFPGATimestamp();
      m_rainbowFirstPixelHue = 0;
      m_manualMode = false;
      m_led2to5State = false;
      m_led8to10Counter = 0;
      m_useBindingSet1 = true;

      // Startup test: set all LEDs to bright white
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, LEDConstants.kStartupTestR, LEDConstants.kStartupTestG, LEDConstants.kStartupTestB);
      }
      m_led.setData(m_ledBuffer);

      DriverStation.reportWarning("LED Subsystem initialized on PWM port " + LEDConstants.kLEDPort + " with " + LEDConstants.kLEDCount + " LEDs - Watch for white flash!", false);
    } catch (Exception e) {
      DriverStation.reportError("LED Subsystem failed to initialize: " + e.getMessage(), e.getStackTrace());
      throw e;
    }
  }

  @Override
  public void periodic() {
    if (!m_manualMode) {
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
      m_ledBuffer.setRGB(index, g, r, b);  // GRB color order for AndyMark LEDs
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
    double elapsed = (currentTime - m_animationStartTime) * 1000;

    double breatheCycle = (elapsed / LEDConstants.kBreatheSpeed) % (2 * Math.PI);
    double brightness = (Math.sin(breatheCycle) + 1.0) / 2.0;

    int r = (int)(m_currentColor[0] * brightness);
    int g = (int)(m_currentColor[1] * brightness);
    int b = (int)(m_currentColor[2] * brightness);

    setAllLEDs(r, g, b);
  }

  /**
   * Updates the rainbow animation
   */
  private void updateRainbow() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      final int hue = (m_rainbowFirstPixelHue + (i * LEDConstants.kRainbowHueModulo / m_ledBuffer.getLength())) % LEDConstants.kRainbowHueModulo;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += LEDConstants.kRainbowHueIncrement;
    m_rainbowFirstPixelHue %= LEDConstants.kRainbowHueModulo;
  }

  /**
   * Updates the chase animation (moving dot)
   */
  private void updateChase() {
    double currentTime = Timer.getFPGATimestamp();
    double elapsed = (currentTime - m_animationStartTime) * 1000;

    int position = (int)((elapsed / LEDConstants.kBlinkSpeed) % m_ledBuffer.getLength());

    setAllLEDs(0, 0, 0);

    for (int i = 0; i < LEDConstants.kChaseTailLength; i++) {
      int ledIndex = (position - i + m_ledBuffer.getLength()) % m_ledBuffer.getLength();
      double brightness = 1.0 - (i * LEDConstants.kChaseBrightnessFade);
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

  public void enableManualMode() {
    if (!m_manualMode) {
      m_manualMode = true;
    }
  }

  public void disableManualMode() {
    m_manualMode = false;
  }

  // ========== BINDING SET 1 METHODS ==========

  public void setLED1Green() {
    System.out.println(">>> A BUTTON PRESSED - Setting LED 1 to green <<<");
    DriverStation.reportWarning(">>> A BUTTON PRESSED - Setting LED 1 to green <<<", false);
    enableManualMode();
    setLED(0, 0, 255, 0);
  }

  public void clearLED1() {
    setLED(0, 0, 0, 0);
  }

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

  public void cycleLED8to10() {
    enableManualMode();

    m_led8to10Counter = (m_led8to10Counter + 1) % LEDConstants.kCycleCounterModulo;

    for (int i = 7; i <= 9; i++) {
      setLED(i, 0, 0, 0);
    }

    if (m_led8to10Counter > 0) {
      for (int i = 0; i < m_led8to10Counter; i++) {
        setLED(7 + i, 0, 255, 0);
      }
    }
  }

  // ========== BINDING SET 2 METHODS (Currently inactive) ==========

  public void bindingSet2_A() {
  }

  public void bindingSet2_A_Release() {
  }

  public void bindingSet2_B() {
  }

  public void bindingSet2_DpadUp() {
  }

  public void turnOffAllAndReset() {
    System.out.println(">>> Y BUTTON PRESSED - Turning off all LEDs and resetting state <<<");
    enableManualMode();

    m_led2to5State = false;
    m_led8to10Counter = 0;

    for (int i = 0; i < LEDConstants.kLEDCount; i++) {
      setLED(i, 0, 0, 0);
    }
  }

  public void toggleBindingSet() {
    m_useBindingSet1 = !m_useBindingSet1;
    System.out.println(">>> Switched to binding set " + (m_useBindingSet1 ? "1" : "2") + " <<<");
    DriverStation.reportWarning("Switched to LED binding set " + (m_useBindingSet1 ? "1" : "2"), false);
  }

  public boolean isBindingSet1() {
    return m_useBindingSet1;
  }

  public void setAllLEDsRed() {
    disableManualMode();
    m_currentPattern = LEDPattern.SOLID;
    m_currentColor = LEDConstants.kRedColor;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 255, 0); // GRB color order
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

    for (int i = 0; i < LEDConstants.kLEDCount; i++) {
      setLED(i, 0, 0, 0);
    }

    for (Integer tagID : tagIDs) {
      if (tagID >= 1 && tagID <= LEDConstants.kLEDCount) {
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

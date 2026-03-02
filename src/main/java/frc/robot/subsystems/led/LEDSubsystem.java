// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import java.util.List;

/**
 * Simple LED subsystem for controlling addressable LEDs.
 * Displays green LEDs based on detected AprilTag IDs.
 */
public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

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
    // Update LED strip with current buffer state
    m_led.setData(m_ledBuffer);
  }

  /**
   * Sets LEDs to green based on detected AprilTag IDs.
   * Each LED position corresponds to an AprilTag ID (LED 0 = Tag 1, LED 1 = Tag 2, etc.)
   *
   * @param tagIDs List of detected AprilTag IDs
   */
  public void setAprilTagLEDs(List<Integer> tagIDs) {
    // Clear all LEDs first
    clearAllLEDs();

    // Light up LEDs corresponding to detected tags in green
    for (Integer tagID : tagIDs) {
      // Map tag ID to LED index (Tag 1 = LED 0, Tag 2 = LED 1, etc.)
      int ledIndex = tagID - 1;

      // Ensure the LED index is within valid range
      if (ledIndex >= 0 && ledIndex < LEDConstants.kLEDCount) {
        setLED(ledIndex, LEDConstants.kGreenColor);
      }
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

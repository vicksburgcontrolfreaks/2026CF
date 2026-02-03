// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.Constants.LEDConstants;

import java.util.List;

/**
 * Command that continuously updates LEDs based on detected AprilTags.
 * Each LED index corresponds to an AprilTag ID (LED 0 = Tag 1, LED 1 = Tag 2, etc.)
 */
public class AprilTagLEDCommand extends Command {
  private final LEDSubsystem m_ledSubsystem;
  private final PhotonVisionSubsystem m_visionSubsystem;
  private final int[] m_color;

  /**
   * Creates a new AprilTagLEDCommand with a specified color.
   *
   * @param ledSubsystem LED subsystem
   * @param visionSubsystem PhotonVision subsystem
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  public AprilTagLEDCommand(
      LEDSubsystem ledSubsystem,
      PhotonVisionSubsystem visionSubsystem,
      int r, int g, int b) {
    m_ledSubsystem = ledSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_color = new int[]{r, g, b};

    addRequirements(ledSubsystem);
  }

  /**
   * Creates a new AprilTagLEDCommand with green color (default).
   *
   * @param ledSubsystem LED subsystem
   * @param visionSubsystem PhotonVision subsystem
   */
  public AprilTagLEDCommand(
      LEDSubsystem ledSubsystem,
      PhotonVisionSubsystem visionSubsystem) {
    this(ledSubsystem, visionSubsystem,
         LEDConstants.kGreenColor[0],
         LEDConstants.kGreenColor[1],
         LEDConstants.kGreenColor[2]);
  }

  @Override
  public void initialize() {
    // Clear all LEDs when command starts
    m_ledSubsystem.clearAprilTagLEDs();
  }

  @Override
  public void execute() {
    // Get detected AprilTag IDs from vision subsystem
    List<Integer> detectedTags = m_visionSubsystem.getDetectedAprilTagIDs();

    // Update LEDs based on detected tags
    m_ledSubsystem.setAprilTagLEDs(detectedTags, m_color);
  }

  @Override
  public void end(boolean interrupted) {
    // Clear all LEDs when command ends
    m_ledSubsystem.clearAprilTagLEDs();
  }

  @Override
  public boolean isFinished() {
    return false; // Run continuously until interrupted
  }
}

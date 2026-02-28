// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

import java.util.List;

/**
 * Command that displays green LEDs based on detected AprilTags.
 * Each LED index corresponds to an AprilTag ID (LED 0 = Tag 1, LED 1 = Tag 2, etc.)
 * Runs continuously while the robot is active.
 */
public class AprilTagLEDCommand extends Command {
  private final LEDSubsystem m_ledSubsystem;
  private final PhotonVisionSubsystem m_visionSubsystem;

  /**
   * Creates a new AprilTagLEDCommand.
   *
   * @param ledSubsystem LED subsystem
   * @param visionSubsystem PhotonVision subsystem
   */
  public AprilTagLEDCommand(
      LEDSubsystem ledSubsystem,
      PhotonVisionSubsystem visionSubsystem) {
    m_ledSubsystem = ledSubsystem;
    m_visionSubsystem = visionSubsystem;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    // Clear all LEDs when command starts
    m_ledSubsystem.clearAllLEDs();
  }

  @Override
  public void execute() {
    // Get detected AprilTag IDs from vision subsystem
    List<Integer> detectedTags = m_visionSubsystem.getDetectedAprilTagIDs();

    // Update LEDs based on detected tags (displays green for each detected tag)
    m_ledSubsystem.setAprilTagLEDs(detectedTags);
  }

  @Override
  public void end(boolean interrupted) {
    // Clear all LEDs when command ends
    m_ledSubsystem.clearAllLEDs();
  }

  @Override
  public boolean isFinished() {
    return false; // Run continuously until interrupted
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * This command prints the raw absolute encoder positions for calibration.
 * Use this to determine the module offsets:
 *
 * 1. Manually align all wheels straight forward
 * 2. Run this command
 * 3. Copy the printed values to Constants.SwerveConstants module offsets
 */
public class PrintModuleOffsets extends Command {
  private final SwerveDrive m_swerveDrive;

  public PrintModuleOffsets(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  @Override
  public void initialize() {
    System.out.println("\n=== SWERVE MODULE CALIBRATION ===");
    System.out.println("Align all wheels straight forward, then copy these values to Constants.java:");
    System.out.println("");
  }

  @Override
  public void execute() {
    // This would require adding methods to SwerveDrive to expose module raw encoder values
    // For now, the values are visible in SmartDashboard
    System.out.println("Check SmartDashboard for absolute encoder values:");
    System.out.println("- FL Absolute Encoder");
    System.out.println("- FR Absolute Encoder");
    System.out.println("- BL Absolute Encoder");
    System.out.println("- BR Absolute Encoder");
    System.out.println("\nUpdate these values in Constants.SwerveConstants:");
    System.out.println("  kFrontLeftOffset = [value];");
    System.out.println("  kFrontRightOffset = [value];");
    System.out.println("  kBackLeftOffset = [value];");
    System.out.println("  kBackRightOffset = [value];");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

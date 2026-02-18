// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.RotateToTargetCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Alliance-aware autonomous command that drives forward and then rotates to face
 * the alliance-specific target based on the FRC Driver Station app color selection.
 *
 * This command executes in sequence:
 * 1. Drive forward a specified distance
 * 2. Rotate to face the alliance target (Blue or Red)
 *
 * The target coordinates are defined in AutoConstants:
 * - Blue Alliance: kBlueTargetX, kBlueTargetY
 * - Red Alliance: kRedTargetX, kRedTargetY
 */
public class DriveAndAlignCommand extends SequentialCommandGroup {

  /**
   * Creates a new DriveAndAlignCommand with default parameters.
   * Drives forward 2.0 meters at 0.5 m/s, then aligns to alliance target.
   *
   * @param swerveDrive The swerve drive subsystem to use for movement
   */
  public DriveAndAlignCommand(SwerveDriveSubsystem swerveDrive) {
    this(swerveDrive, 1.0, 1);
  }

  /**
   * Creates a new DriveAndAlignCommand with custom drive parameters.
   *
   * @param swerveDrive The swerve drive subsystem to use for movement
   * @param driveDistanceMeters Distance to drive forward in meters
   * @param driveSpeedMetersPerSecond Speed to drive at in m/s
   */
  public DriveAndAlignCommand(SwerveDriveSubsystem swerveDrive,
      double driveDistanceMeters,
      double driveSpeedMetersPerSecond) {

    // Get the alliance color from the FRC Driver Station
    // If no alliance color is set, defaults to Blue
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Determine which target coordinates to use based on alliance color
    double targetX;
    double targetY;

    if (alliance == Alliance.Red) {
      // Use red alliance target coordinates
      targetX = AutoConstants.kRedTargetX;
      targetY = AutoConstants.kRedTargetY;
      System.out.println("DriveAndAlign: Red Alliance - targeting (" + targetX + ", " + targetY + ")");
    } else {
      // Use blue alliance target coordinates (default)
      targetX = AutoConstants.kBlueTargetX;
      targetY = AutoConstants.kBlueTargetY;
      System.out.println("DriveAndAlign: Blue Alliance - targeting (" + targetX + ", " + targetY + ")");
    }

    // Build the command sequence
    addCommands(
      // Step 1: Drive forward the specified distance at the specified speed
      new DriveForwardCommand(swerveDrive, driveDistanceMeters, driveSpeedMetersPerSecond),

      // Step 2: Rotate to face the alliance-specific target
      new RotateToTargetCommand(swerveDrive, targetX, targetY)
    );
  }
}

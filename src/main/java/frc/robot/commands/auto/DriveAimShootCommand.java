// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.RotateToTargetCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

/**
 * Autonomous command that:
 * 1. Rotates to face the speaker
 * 2. Drives toward the speaker until within 2 meters
 * 3. Shoots for several seconds
 */
public class DriveAimShootCommand extends SequentialCommandGroup {

  /**
   * Creates autonomous command to drive to speaker and shoot
   *
   * @param swerveDrive The swerve drive subsystem
   * @param vision The vision subsystem for distance measurement
   * @param shooter The shooter subsystem
   */
  public DriveAimShootCommand(
      SwerveDriveSubsystem swerveDrive,
      PhotonVisionSubsystem vision,
      ShooterSubsystem shooter) {

    // Get the alliance color from the FRC Driver Station
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Determine which target coordinates to use based on alliance color
    double targetX;
    double targetY;

    if (alliance == Alliance.Red) {
      targetX = AutoConstants.kRedTargetX;
      targetY = AutoConstants.kRedTargetY;
      System.out.println("Auto: Red Alliance - targeting speaker at (" + targetX + ", " + targetY + ")");
    } else {
      targetX = AutoConstants.kBlueTargetX;
      targetY = AutoConstants.kBlueTargetY;
      System.out.println("Auto: Blue Alliance - targeting speaker at (" + targetX + ", " + targetY + ")");
    }

    addCommands(
      // Step 1: Rotate to face the speaker target
      new RotateToTargetCommand(swerveDrive, targetX, targetY),

      // Step 2: Drive toward speaker until within 2 meters
      Commands.run(() -> {
        // Drive forward slowly toward target
        swerveDrive.drive(0.3, 0, 0, true); // 0.3 m/s forward, field-relative
      }, swerveDrive)
      .until(() -> {
        double distance = vision.getDistanceToSpeaker();
        // Stop when within 2 meters or if vision is lost
        return distance > 0 && distance <= 2.0;
      })
      .withTimeout(5.0), // Safety timeout: 5 seconds max drive time

      // Step 3: Stop driving
      Commands.runOnce(() -> {
        swerveDrive.drive(0, 0, 0, true);
      }, swerveDrive),

      // Step 4: Fine-tune rotation to face speaker
      new RotateToTargetCommand(swerveDrive, targetX, targetY),

      // Step 5: Spin up shooter and shoot for 3 seconds
      Commands.run(() -> {
        // Get distance and calculate RPM
        double distance = vision.getDistanceToSpeaker();
        double targetRPM = ShooterConstants.getRPMForDistance(distance);

        // Run shooter at calculated RPM
        shooter.runAllMotors(targetRPM);
      }, shooter)
      .withTimeout(3.0), // Shoot for 3 seconds

      // Step 6: Stop shooter
      Commands.runOnce(() -> {
        shooter.stopAll();
      }, shooter)
    );
  }
}

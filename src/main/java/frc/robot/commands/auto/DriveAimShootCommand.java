// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.RotateToTargetCommand;
import frc.robot.commands.shooter.ShooterSequenceCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
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
      DriveSubsystem swerveDrive,
      PhotonVisionSubsystem vision,
      ShooterSubsystem shooter) {

    // Get the alliance color from the FRC Driver Station
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    Translation2d target;

    if (alliance == Alliance.Red) {
      target = AutoConstants.redTarget;
    } else {
      target = AutoConstants.blueTarget;
    }

    addCommands(
      Commands.run(() -> shooter.runIndexer(true, true), shooter).withTimeout(1.5),

      Commands.run(() -> {
        swerveDrive.drive(-0.3, 0, 0, true);
      }, swerveDrive)
      .until(() -> {
        double distance = vision.getDistanceToSpeaker();
        return distance > 0 && distance <= 2.0;
      })
      .withTimeout(1),
      Commands.runOnce(() -> {
        swerveDrive.drive(0, 0, 0, true);
      }, swerveDrive),

      new RotateToTargetCommand(swerveDrive, target),

      new ShooterSequenceCommand(shooter, vision).withTimeout(5.0)
    );
  }
}

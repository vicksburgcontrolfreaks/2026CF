// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.collector.ExtendHopperCommand;
import frc.robot.commands.shooter.ShooterWithAutoAimCommand;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * ExtendBackupAndShootCommand - Autonomous command that:
 * 1. Extends the hopper
 * 2. Backs up 1 meter slowly
 * 3. Aims at the speaker target and shoots for 5 seconds
 */
public class ExtendBackupAndShootCommand extends SequentialCommandGroup {

  /**
   * Creates a new ExtendBackupAndShootCommand.
   *
   * @param collector The collector subsystem
   * @param swerveDrive The drive subsystem
   * @param shooter The shooter subsystem
   */
  public ExtendBackupAndShootCommand(
      CollectorSubsystem collector,
      DriveSubsystem swerveDrive,
      ShooterSubsystem shooter) {

    addCommands(
        // Step 1: Extend the hopper
        new ExtendHopperCommand(collector),

        // Step 2: Back up 1 meter slowly (0.5 m/s for 2 seconds = 1 meter)
        Commands.run(
            () -> swerveDrive.drive(-0.5, 0, 0, false),
            swerveDrive
        ).withTimeout(2.0),

        // Step 3: Stop moving
        Commands.runOnce(() -> swerveDrive.stop(), swerveDrive),

        // Step 4: Aim at target and shoot for 5 seconds using existing auto-aim command
        // Pass in zero suppliers for joystick inputs and false for manual override
        new ShooterWithAutoAimCommand(
            shooter,
            swerveDrive,
            () -> 0.0,  // No forward/back movement
            () -> 0.0,  // No left/right movement
            () -> 0.0,  // No manual rotation
            () -> false // Don't override auto-aim
        ).withTimeout(5.0),

        // Step 5: Stop everything
        Commands.runOnce(() -> {
          swerveDrive.stop();
          shooter.StopFloor();
          shooter.StopIndexer();
        }, swerveDrive, shooter)
    );
  }
}

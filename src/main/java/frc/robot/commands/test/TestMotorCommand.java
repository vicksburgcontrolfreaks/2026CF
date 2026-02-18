// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.test.TestMotorSubsystem;

public class TestMotorCommand extends Command {
  private final TestMotorSubsystem testMotors;
  private final int motorNumber;
  private final double speed;

  /**
   * Command to run a specific test motor at a given speed
   * @param testMotors The test motor subsystem
   * @param motorNumber The motor number (16, 17, 18, or 19)
   * @param speed The speed from -1.0 to 1.0
   */
  public TestMotorCommand(TestMotorSubsystem testMotors, int motorNumber, double speed) {
    this.testMotors = testMotors;
    this.motorNumber = motorNumber;
    this.speed = speed;
    addRequirements(testMotors);
  }

  @Override
  public void execute() {
    switch (motorNumber) {
      case 16:
        testMotors.runRightShooter(speed);
        break;
      case 17:
        testMotors.runFloorMotor(speed);
        break;
      case 18:
        testMotors.runIndexer(speed);
        break;
      case 19:
        testMotors.runLeftShooter(speed);
        break;
      default:
        testMotors.stopAll();
    }
  }

  @Override
  public void end(boolean interrupted) {
    testMotors.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

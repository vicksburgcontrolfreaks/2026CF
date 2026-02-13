// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.test;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TestMotorConstants;

public class TestMotorSubsystem extends SubsystemBase {
  // private final SparkMax motor16;
  private final SparkMax motor17;
  private final SparkMax motor18;
  private final SparkMax motor19;

  public TestMotorSubsystem() {
    // motor16 = new SparkMax(TestMotorConstants.kMotor1Id, MotorType.kBrushless);
    motor17 = new SparkMax(TestMotorConstants.kMotor2Id, MotorType.kBrushless);
    motor18 = new SparkMax(TestMotorConstants.kMotor3Id, MotorType.kBrushless);
    motor19 = new SparkMax(TestMotorConstants.kMotor4Id, MotorType.kBrushless);

    // Apply configuration to all motors
    // motor16.configure(TestMotorConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor17.configure(TestMotorConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor18.configure(TestMotorConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor19.configure(TestMotorConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set inversions
    // motor16.setInverted(TestMotorConstants.kMotor1Inverted);
    motor17.setInverted(TestMotorConstants.kMotor2Inverted);
    motor18.setInverted(TestMotorConstants.kMotor3Inverted);
    motor19.setInverted(TestMotorConstants.kMotor4Inverted);

    System.out.println(">>> TestMotorSubsystem initialized with 3 motors (CAN IDs: 17, 18, 19) <<<");
  }

  /**
   * Run motor 16 (D-pad Down)
   * @param speed Speed from -1.0 to 1.0
   */
  // public void runMotor16(double speed) {
  //   motor16.set(speed);
  // }

  /**
   * Run motor 17 (D-pad Left)
   * @param speed Speed from -1.0 to 1.0
   */
  public void runMotor17(double speed) {
    motor17.set(speed);
  }

  /**
   * Run motor 18 (D-pad Right)
   * @param speed Speed from -1.0 to 1.0
   */
  public void runMotor18(double speed) {
    motor18.set(speed);
  }

  /**
   * Run motor 19 (D-pad Up)
   * @param speed Speed from -1.0 to 1.0
   */
  public void runMotor19(double speed) {
    motor19.set(speed);
  }

  /**
   * Stop all motors
   */
  public void stopAll() {
    // motor16.set(0);
    motor17.set(0);
    motor18.set(0);
    motor19.set(0);
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with telemetry
    // SmartDashboard.putNumber("Test/Motor16_Speed", motor16.get());
    SmartDashboard.putNumber("Test/Motor17_Speed", motor17.get());
    SmartDashboard.putNumber("Test/Motor18_Speed", motor18.get());
    SmartDashboard.putNumber("Test/Motor19_Speed", motor19.get());
  }
}

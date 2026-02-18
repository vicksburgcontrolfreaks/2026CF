// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.test;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TestMotorConstants;

public class TestMotorSubsystem extends SubsystemBase {
  private final SparkFlex m_rightShooterMotor;
  private final SparkFlex m_floorMotor;
  private final SparkFlex m_indexerMotor;
  private final SparkFlex m_leftShooterMotor;

  public TestMotorSubsystem() {
    System.out.println(">>> Initializing TestMotorSubsystem <<<");

    System.out.println("Creating right shooter motor (ID 16)...");
    m_rightShooterMotor = new SparkFlex(TestMotorConstants.kRightShooterId, MotorType.kBrushless);
    System.out.println("Creating floor motor (ID 17)...");
    m_floorMotor = new SparkFlex(TestMotorConstants.kFloorMotorId, MotorType.kBrushless);
    System.out.println("Creating indexer motor (ID 18)...");
    m_indexerMotor = new SparkFlex(TestMotorConstants.kIndexerMotorId, MotorType.kBrushless);
    System.out.println("Creating left shooter motor (ID 19)...");
    m_leftShooterMotor = new SparkFlex(TestMotorConstants.kLeftShooterId, MotorType.kBrushless);

    System.out.println("Configuring right shooter motor...");
    m_rightShooterMotor.configure(TestMotorConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    System.out.println("Configuring floor motor...");
    m_floorMotor.configure(TestMotorConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    System.out.println("Configuring indexer motor...");
    m_indexerMotor.configure(TestMotorConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    System.out.println("Configuring left shooter motor...");
    m_leftShooterMotor.configure(TestMotorConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    System.out.println(">>> TestMotorSubsystem initialized with 4 motors (CAN IDs: 16, 17, 18, 19) <<<");
  }

  public void runRightShooter(double speed) {
    m_rightShooterMotor.set(speed);
  }

  public void runFloorMotor(double speed) {
    m_floorMotor.set(speed);
  }

  public void runIndexer(double speed) {
    m_indexerMotor.set(speed);
  }

  public void runLeftShooter(double speed) {
    m_leftShooterMotor.set(speed);
  }

  public void runAllMotors() {
    m_rightShooterMotor.getClosedLoopController().setSetpoint(
      TestMotorConstants.kTargetRPM,
      ControlType.kVelocity
    );

    m_floorMotor.set(-0.1);

    m_indexerMotor.getClosedLoopController().setSetpoint(
      TestMotorConstants.kTargetRPM,
      ControlType.kVelocity
    );

    m_leftShooterMotor.getClosedLoopController().setSetpoint(
      -TestMotorConstants.kTargetRPM,
      ControlType.kVelocity
    );
  }

  public void stopAll() {
    m_rightShooterMotor.set(0);
    m_floorMotor.set(0);
    m_indexerMotor.set(0);
    m_leftShooterMotor.set(0);
  }

  public boolean isAnyMotorOverCurrent(double threshold) {
    return m_rightShooterMotor.getOutputCurrent() > threshold ||
           m_floorMotor.getOutputCurrent() > threshold ||
           m_indexerMotor.getOutputCurrent() > threshold ||
           m_leftShooterMotor.getOutputCurrent() > threshold;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Test/RightShooter_Speed", m_rightShooterMotor.get());
    SmartDashboard.putNumber("Test/FloorMotor_Speed", m_floorMotor.get());
    SmartDashboard.putNumber("Test/Indexer_Speed", m_indexerMotor.get());
    SmartDashboard.putNumber("Test/LeftShooter_Speed", m_leftShooterMotor.get());

    SmartDashboard.putNumber("Test/RightShooter_Current", m_rightShooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("Test/FloorMotor_Current", m_floorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Test/Indexer_Current", m_indexerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Test/LeftShooter_Current", m_leftShooterMotor.getOutputCurrent());

    SmartDashboard.putNumber("Test/RightShooter_Velocity", m_rightShooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Test/FloorMotor_Velocity", m_floorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Test/Indexer_Velocity", m_indexerMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Test/LeftShooter_Velocity", m_leftShooterMotor.getEncoder().getVelocity());
  }
}

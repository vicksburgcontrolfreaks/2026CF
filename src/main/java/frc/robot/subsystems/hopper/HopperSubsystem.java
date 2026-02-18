// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

  private final SparkFlex m_hopperMotor;

  public HopperSubsystem() {
    m_hopperMotor = new SparkFlex(HopperConstants.kMotorId, MotorType.kBrushless);
    m_hopperMotor.configure(
        HopperConstants.config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void deploy() {
    m_hopperMotor.set(HopperConstants.kDeploySpeed);
  }

  public void retract() {
    m_hopperMotor.set(HopperConstants.kRetractSpeed);
  }

  public void stop() {
    m_hopperMotor.set(0);
  }

  public double getCurrent() {
    return m_hopperMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {}
}

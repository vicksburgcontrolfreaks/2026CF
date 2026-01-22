// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConfigConstants;

public class SwerveConfig {

  private static RobotConfig robotConfig = null;

  public static RobotConfig getRobotConfig() {
    if (robotConfig == null) {
      try {
        // Create robot configuration for PathPlanner
        // Mass is in kg, MOI is in kg*m^2
        robotConfig = new RobotConfig(
          SwerveConfigConstants.kRobotMassKg, // Robot mass (kg)
          SwerveConfigConstants.kRobotMOI,  // Robot MOI (kg*m^2)
          new com.pathplanner.lib.config.ModuleConfig(
            SwerveConstants.kWheelCircumferenceMeters / (2 * Math.PI), // Wheel radius (m)
            SwerveConstants.kMaxSpeedMetersPerSecond, // Max wheel speed (m/s)
            SwerveConfigConstants.kWheelCoefficientOfFriction, // Wheel COF - coefficient of friction
            DCMotor.getNEO(SwerveConfigConstants.kNumMotorsPerModule).withReduction(SwerveConstants.kDriveGearRatio), // Drive motor
            SwerveConstants.kDriveMotorCurrentLimit, // Drive current limit (A)
            SwerveConfigConstants.kNumMotorsPerModule // Number of drive motors per module
          ),
          // Module locations (same order as kinematics: FL, FR, BL, BR)
          new Translation2d(SwerveConstants.kWheelBaseMeters / 2.0, SwerveConstants.kTrackWidthMeters / 2.0),  // FL
          new Translation2d(SwerveConstants.kWheelBaseMeters / 2.0, -SwerveConstants.kTrackWidthMeters / 2.0), // FR
          new Translation2d(-SwerveConstants.kWheelBaseMeters / 2.0, SwerveConstants.kTrackWidthMeters / 2.0), // BL
          new Translation2d(-SwerveConstants.kWheelBaseMeters / 2.0, -SwerveConstants.kTrackWidthMeters / 2.0) // BR
        );
      } catch (Exception e) {
        e.printStackTrace();
        throw new RuntimeException("Failed to create RobotConfig for PathPlanner", e);
      }
    }
    return robotConfig;
  }

  public static PPHolonomicDriveController getHolonomicController() {
    return new PPHolonomicDriveController(
      new PIDConstants(AutoConstants.kPTranslation, AutoConstants.kITranslation, AutoConstants.kDTranslation),
      new PIDConstants(AutoConstants.kPRotation, AutoConstants.kIRotation, AutoConstants.kDRotation)
    );
  }
}

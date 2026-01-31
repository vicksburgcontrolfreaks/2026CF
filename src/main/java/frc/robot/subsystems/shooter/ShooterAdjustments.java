// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ShooterAdjustments extends SubsystemBase {
  private final SparkMax m_shooterTop;
  private final SparkMax m_shooterFront;
  private final SwerveDrive m_swerveDrive;

  // Current speed of the front shooter (starts at initial speed)
  private double m_frontShooterSpeed;

  // Flag to track if shooter is running
  private boolean m_isRunning;

  /** Creates a new ShooterAdjustments subsystem. */
  public ShooterAdjustments(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;

    // Initialize the shooter motors
    m_shooterTop = new SparkMax(SwerveConstants.kShooterTop, MotorType.kBrushless);
    m_shooterFront = new SparkMax(SwerveConstants.kShooterFront, MotorType.kBrushless);

    // Configure motors using the config object
    m_shooterTop.configure(Configs.Shooter.shooterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_shooterFront.configure(Configs.Shooter.shooterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Initialize state
    m_frontShooterSpeed = ShooterConstants.kFrontShooterStartSpeed;
    m_isRunning = false;
  }

  /**
   * Starts the shooter motors.
   * Power is calculated based on distance to the nearest target.
   * Both motors use the same power level based on range.
   */
  public void startShooter() {
    m_isRunning = true;

    // Calculate distance to nearest target
    double distance = getDistanceToNearestTarget();

    // Calculate power based on distance
    double power = calculateShooterPower(distance);

    // Set both shooters to the calculated power
    m_shooterTop.set(power);
    m_shooterFront.set(power);
    m_frontShooterSpeed = power;

    System.out.printf("Shooter: Distance to target = %.2fm, Power = %.2f%%\n",
                      distance, power * 100);
  }

  /**
   * Stops both shooter motors and resets the front shooter speed.
   */
  public void stopShooter() {
    m_isRunning = false;
    m_shooterTop.set(0.0);
    m_shooterFront.set(0.0);
    // Reset front shooter speed for next run
    m_frontShooterSpeed = ShooterConstants.kFrontShooterStartSpeed;
  }

  /**
   * Gets whether the shooter is currently running.
   */
  public boolean isRunning() {
    return m_isRunning;
  }

  /**
   * Gets the current front shooter speed.
   */
  public double getFrontShooterSpeed() {
    return m_frontShooterSpeed;
  }

  /**
   * Calculate distance to the nearest target (red or blue).
   * @return Distance to the nearest target in meters
   */
  public double getDistanceToNearestTarget() {
    Translation2d robotPosition = m_swerveDrive.getPose().getTranslation();
    Translation2d redTarget = new Translation2d(AutoConstants.kRedTargetX, AutoConstants.kRedTargetY);
    Translation2d blueTarget = new Translation2d(AutoConstants.kBlueTargetX, AutoConstants.kBlueTargetY);

    double distanceToRed = robotPosition.getDistance(redTarget);
    double distanceToBlue = robotPosition.getDistance(blueTarget);

    return Math.min(distanceToRed, distanceToBlue);
  }

  /**
   * Calculate shooter power based on distance to target.
   * Uses a linear interpolation between min and max power based on distance.
   *
   * @param distance Distance to target in meters
   * @return Power level between kMinShooterPower and kMaxShooterPower
   */
  public double calculateShooterPower(double distance) {
    // Clamp distance to valid range
    if (distance <= ShooterConstants.kMinShooterDistance) {
      return ShooterConstants.kMinShooterPower;
    }
    if (distance >= ShooterConstants.kMaxShooterDistance) {
      return ShooterConstants.kMaxShooterPower;
    }

    // Linear interpolation: power = minPower + (distance - minDist) * slope
    double slope = (ShooterConstants.kMaxShooterPower - ShooterConstants.kMinShooterPower) /
                   (ShooterConstants.kMaxShooterDistance - ShooterConstants.kMinShooterDistance);

    return ShooterConstants.kMinShooterPower +
           (distance - ShooterConstants.kMinShooterDistance) * slope;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (every 20ms)

    // If shooter is running, continuously update power based on distance
    // This allows dynamic adjustment as the robot moves
    if (m_isRunning) {
      double distance = getDistanceToNearestTarget();
      double power = calculateShooterPower(distance);

      // Update both motors with the calculated power
      m_shooterTop.set(power);
      m_shooterFront.set(power);
      m_frontShooterSpeed = power;
    }
  }
}

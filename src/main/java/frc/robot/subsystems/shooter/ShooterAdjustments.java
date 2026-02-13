// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Shooter subsystem with indexer motor.
 * The shooter has two flywheel motors and an indexer motor that feeds balls into the shooter.
 * The indexer only runs after the shooter wheels have had time to spin up.
 */
public class ShooterAdjustments extends SubsystemBase {
  private final SparkMax m_shooterTop;
  private final SparkMax m_shooterFront;
  private final SparkMax m_indexer;
  private final SwerveDriveSubsystem m_swerveDrive;

  // Current power of the front shooter (starts at initial power)
  private double m_frontShooterPower;

  // Flag to track if shooter is running
  private boolean m_isRunning;

  // Timer to track shooter spin-up time
  private final Timer m_spinUpTimer = new Timer();

  /** Creates a new ShooterAdjustments subsystem. */
  public ShooterAdjustments(SwerveDriveSubsystem swerveDrive) {
    m_swerveDrive = swerveDrive;

    // Initialize the shooter motors
    m_shooterTop = new SparkMax(ShooterConstants.kTopMotorId, MotorType.kBrushless);
    m_shooterFront = new SparkMax(ShooterConstants.kBottomMotorId, MotorType.kBrushless);
    m_indexer = new SparkMax(ShooterConstants.kIndexerMotorId, MotorType.kBrushless);

    // Configure motors using the config object
    m_shooterTop.configure(Configs.Shooter.shooterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_shooterFront.configure(Configs.Shooter.shooterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_indexer.configure(Configs.Shooter.shooterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Initialize state
    m_frontShooterPower = ShooterConstants.kFrontShooterStartPower;
    m_isRunning = false;
  }

  /**
   * Starts the shooter motors.
   * Power is calculated based on distance to the nearest target.
   * Both motors use the same power level based on range.
   * Indexer will start after spin-up time.
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
    m_frontShooterPower = power;

    // Reset and start spin-up timer
    m_spinUpTimer.restart();

    System.out.printf("Shooter: Distance to target = %.2fm, Power = %.2f%%, Spin-up starting\n",
                      distance, power * 100);
  }

  /**
   * Stops all shooter motors (including indexer) and resets state.
   */
  public void stopShooter() {
    m_isRunning = false;
    m_shooterTop.set(0.0);
    m_shooterFront.set(0.0);
    m_indexer.set(0.0);
    m_spinUpTimer.stop();
    m_spinUpTimer.reset();
    // Reset front shooter speed for next run
    m_frontShooterPower = ShooterConstants.kFrontShooterStartPower;
  }

  /**
   * Gets whether the shooter is currently running.
   */
  public boolean isRunning() {
    return m_isRunning;
  }

  /**
   * Gets whether the shooter has spun up and is ready to feed balls.
   * @return true if spin-up time has elapsed
   */
  public boolean isReadyToFeed() {
    return m_isRunning && m_spinUpTimer.hasElapsed(ShooterConstants.kSpinUpTimeSeconds);
  }

  /**
   * Gets the current front shooter power.
   */
  public double getFrontShooterPower() {
    return m_frontShooterPower;
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

      // Update both shooter motors with the calculated power
      m_shooterTop.set(power);
      m_shooterFront.set(power);
      m_frontShooterPower = power;

      // Only run indexer after shooter has spun up
      if (isReadyToFeed()) {
        m_indexer.set(ShooterConstants.kIndexerSpeed);
      } else {
        m_indexer.set(0.0);
      }
    }
  }
}

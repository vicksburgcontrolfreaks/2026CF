// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveCommandConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveDriveCommand extends Command {
  private final SwerveDrive m_swerveDrive;
  private final DoubleSupplier m_xSupplier;
  private final DoubleSupplier m_ySupplier;
  private final DoubleSupplier m_rotSupplier;
  private final DoubleSupplier m_speedLimitSupplier;

  // Slew rate limiters to make joystick inputs smoother
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(DriveCommandConstants.kTranslationSlewRate);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(DriveCommandConstants.kTranslationSlewRate);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveCommandConstants.kRotationSlewRate);

  public SwerveDriveCommand(
      SwerveDrive swerveDrive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      DoubleSupplier speedLimitSupplier) {
    m_swerveDrive = swerveDrive;
    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;
    m_rotSupplier = rotSupplier;
    m_speedLimitSupplier = speedLimitSupplier;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    // Reset limiters when command starts
    m_xLimiter.reset(0);
    m_yLimiter.reset(0);
    m_rotLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Get joystick inputs
    double xInput = m_xSupplier.getAsDouble();
    double yInput = m_ySupplier.getAsDouble();
    double rotInput = m_rotSupplier.getAsDouble();

    // Apply deadband
    xInput = MathUtil.applyDeadband(xInput, OperatorConstants.kDeadband);
    yInput = MathUtil.applyDeadband(yInput, OperatorConstants.kDeadband);
    rotInput = MathUtil.applyDeadband(rotInput, OperatorConstants.kDeadband);

    // Apply slew rate limiter for smoother control
    xInput = m_xLimiter.calculate(xInput);
    yInput = m_yLimiter.calculate(yInput);
    rotInput = m_rotLimiter.calculate(rotInput);

    // Get speed limit (default to normal if supplier returns 0)
    double speedLimit = m_speedLimitSupplier.getAsDouble();
    if (speedLimit == 0) {
      speedLimit = OperatorConstants.kNormalSpeedLimit;
    }

    // Calculate speeds and apply speed limit
    double xSpeed = xInput * SwerveConstants.kMaxSpeedMetersPerSecond * speedLimit;
    double ySpeed = yInput * SwerveConstants.kMaxSpeedMetersPerSecond * speedLimit;
    double rot = rotInput * SwerveConstants.kMaxAngularSpeedRadiansPerSecond * speedLimit;

    // Drive the robot
    m_swerveDrive.drive(xSpeed, ySpeed, rot, m_swerveDrive.isFieldOriented());
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

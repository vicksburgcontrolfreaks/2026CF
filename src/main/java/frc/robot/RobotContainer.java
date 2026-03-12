// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.collector.RunCollectorCommand;
import frc.robot.commands.collector.StopCollectorCommand;
import frc.robot.commands.collector.hopper.ManualExtendHopperCommand;
import frc.robot.commands.collector.hopper.ManualRetractHopperCommand;
import frc.robot.commands.drive.RotateToTargetCommand;
import frc.robot.commands.led.AprilTagLEDCommand;
import frc.robot.commands.shooter.AdvancedShooterCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.commands.auto.DriveAimShootCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

public class RobotContainer {
  private final DriveSubsystem m_swerveDrive = new DriveSubsystem();
  private final PhotonVisionSubsystem m_visionSubsystem = new PhotonVisionSubsystem(m_swerveDrive);
  private final LEDSubsystem m_ledSubsystem = null;
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final CollectorSubsystem m_collector = new CollectorSubsystem();
  private final ClimberSubsystem m_climber = null;

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_mechanismController;

  private final Command m_autoCommand;

  public RobotContainer() {
      m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
      m_mechanismController = new CommandXboxController(OIConstants.kMechanismControllerPort);

      // Set vision subsystem for continuous target RPM calculation
      m_shooterSubsystem.setVisionSubsystem(m_visionSubsystem);

      m_autoCommand = null;

      configureDefaultCommands();
      configureDriverBindings();
      configureMechanismBindings();
  }

  private void configureDefaultCommands() {
    m_swerveDrive.setDefaultCommand(
      new RunCommand(
        () -> {
          double speedMultiplier;
          if (m_driverController.rightBumper().getAsBoolean()) {
            speedMultiplier = OIConstants.kTurboSpeedLimit;
          } else if (m_driverController.leftBumper().getAsBoolean()) {
            speedMultiplier = OIConstants.kPrecisionSpeedLimit;
          } else {
            speedMultiplier = OIConstants.kNormalSpeedLimit;
          }

          m_swerveDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond * speedMultiplier,
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond * speedMultiplier,
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxAngularSpeed * speedMultiplier,
            true);
        },
        m_swerveDrive)
    );

    // No default command for shooter - let periodic() handle continuous RPM calculation
    // Motors stay idle until AdvancedShooterCommand is triggered

    if (m_visionSubsystem != null && m_ledSubsystem != null) {
      m_ledSubsystem.setDefaultCommand(
        new AprilTagLEDCommand(m_ledSubsystem, m_visionSubsystem)
      );
    }
  }

  private void configureDriverBindings() {
    m_driverController.start().onTrue(
      m_swerveDrive.runOnce(() -> m_swerveDrive.zeroHeading())
    );

    m_driverController.y().onTrue(
      Commands.either(
        new RotateToTargetCommand(m_swerveDrive, AutoConstants.blueTarget),
        new RotateToTargetCommand(m_swerveDrive, AutoConstants.redTarget),
        () -> DriverStation.getAlliance().isPresent() &&
              DriverStation.getAlliance().get() == Alliance.Blue
      )
    );
  }

  private void configureMechanismBindings() {
    m_mechanismController.a().whileTrue(
      new RunCollectorCommand(m_collector, false).alongWith(Commands.run(() -> {
        m_shooterSubsystem.runFloor(false); m_shooterSubsystem.runIndexerSlowReverse(); }, m_shooterSubsystem))
    );

    m_mechanismController.b().onTrue(
        new StopCollectorCommand(m_collector).alongWith(
            Commands.runOnce(() -> {
            m_shooterSubsystem.StopFloor();
            m_shooterSubsystem.StopIndexer();
          }, m_shooterSubsystem)
        )
    );

    m_mechanismController.x().onTrue(
      new RunCollectorCommand(m_collector, true)
        .alongWith(Commands.run(() -> m_shooterSubsystem.runFloor(true), m_shooterSubsystem))
    );

    m_mechanismController.y().onTrue(
      Commands.run(() -> m_shooterSubsystem.runIndexer(true, true), m_shooterSubsystem).withTimeout(0.5)
    );

    m_mechanismController.rightTrigger().whileTrue(
      new AdvancedShooterCommand(m_shooterSubsystem, m_visionSubsystem)
    );

  /* 
    m_mechanismController.povUp().whileTrue(
      Commands.run(() -> m_climber.setSpeed(0.2), m_climber)
        .finallyDo(() -> m_climber.stop())
    );

    m_mechanismController.povDown().whileTrue(
      Commands.run(() -> m_climber.setSpeed(-0.2), m_climber)
        .finallyDo(() -> m_climber.stop())
    );
  */
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  public DriveSubsystem getSwerveDrive() {
    return m_swerveDrive;
  }

  public PhotonVisionSubsystem getVisionSubsystem() {
    return m_visionSubsystem;
  }

  public LEDSubsystem getLEDSubsystem() {
    return m_ledSubsystem;
  }
}

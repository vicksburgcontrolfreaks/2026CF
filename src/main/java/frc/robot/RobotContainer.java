// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.BackupAndShootCommand;
import frc.robot.commands.auto.DriveAimShootCommand;
import frc.robot.commands.collector.RunCollectorCommand;
import frc.robot.commands.collector.StopCollectorCommand;
import frc.robot.commands.drive.RotateToTargetCommand;
import frc.robot.commands.led.AprilTagLEDCommand;
import frc.robot.commands.shooter.AdvancedShooterCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import choreo.auto.AutoFactory;

public class RobotContainer {
  private final DriveSubsystem m_swerveDrive = new DriveSubsystem();
  private final PhotonVisionSubsystem m_visionSubsystem = new PhotonVisionSubsystem(m_swerveDrive);
  private final LEDSubsystem m_ledSubsystem = null;
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(m_visionSubsystem);
  private final CollectorSubsystem m_collector = new CollectorSubsystem();
  private final ClimberSubsystem m_climber = null;

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_mechanismController;

  private final AutoFactory autoFactory;
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public RobotContainer() {
      m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
      m_mechanismController = new CommandXboxController(OIConstants.kMechanismControllerPort);

      autoFactory = new AutoFactory(
          m_swerveDrive::getPose,
          m_swerveDrive::resetOdometry,
          m_swerveDrive::followTrajectory,
          false,  // Don't mirror - Red_Right_Loop is already designed for red alliance
          m_swerveDrive
      );

      configureAutos();

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

  /**
   * Configures autonomous commands and adds them to the chooser.
   * The chooser will appear in Shuffleboard/SmartDashboard for selection.
   */
  private void configureAutos() {
    // Option 1: Simple Backup and Shoot (DEFAULT)
    Command backupAndShoot = new BackupAndShootCommand(m_swerveDrive, m_shooterSubsystem);
    m_autoChooser.setDefaultOption("Backup and Shoot", backupAndShoot);

    // Option 2: Drive Aim Shoot
    Command driveAimShoot = new DriveAimShootCommand(m_swerveDrive, m_visionSubsystem, m_shooterSubsystem);
    m_autoChooser.addOption("Drive Aim Shoot", driveAimShoot);

    // Option 3: Red Right Loop (Choreo trajectory)
    try {
      Command trajectoryCommand = autoFactory.trajectoryCmd("RedRightLoop");
      m_autoChooser.addOption("Red Right Loop", trajectoryCommand);
    } catch (Exception e) {
      System.err.println("Failed to load Choreo trajectory 'RedRightLoop': " + e.getMessage());
      System.err.println("Make sure you have .traj files in src/main/deploy/choreo/");
      e.printStackTrace();
    }

    // Option 4: Do Nothing
    m_autoChooser.addOption("Do Nothing", Commands.none());

    // Add the chooser to the dashboard
    SmartDashboard.putData("Auto Mode", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
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

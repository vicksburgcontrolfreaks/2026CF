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
import frc.robot.commands.auton.ExtendBackupAndShootCommand;
import frc.robot.commands.collector.RunCollectorCommand;
import frc.robot.commands.collector.StopCollectorCommand;
import frc.robot.commands.collector.ExtendHopperCommand;
import frc.robot.commands.collector.RetractHopperCommand;
import frc.robot.commands.collector.HopperHalfwaySequenceCommand;
import frc.robot.commands.drive.RotateToTargetCommand;
import frc.robot.commands.shooter.ShooterWithAutoAimCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


public class RobotContainer {
  private final DriveSubsystem m_swerveDrive = new DriveSubsystem();
  private final PhotonVisionSubsystem m_visionSubsystem = new PhotonVisionSubsystem(m_swerveDrive);
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(m_visionSubsystem);
  private final CollectorSubsystem m_collector = new CollectorSubsystem();

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_mechanismController;

  private SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
      m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
      m_mechanismController = new CommandXboxController(OIConstants.kMechanismControllerPort);

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

          // For Red alliance, flip X/Y inputs so "forward" drives toward blue alliance (X=0)
          // Blue alliance: 0° = forward toward red side (positive X)
          // Red alliance: 180° = forward toward blue side (negative X)
          boolean isRed = DriverStation.getAlliance().isPresent() &&
                          DriverStation.getAlliance().get() == Alliance.Red;
          double allianceFlip = isRed ? -1.0 : 1.0;

          m_swerveDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * m_swerveDrive.getMaxSpeedMetersPerSecond() * speedMultiplier * allianceFlip,
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * m_swerveDrive.getMaxSpeedMetersPerSecond() * speedMultiplier * allianceFlip,
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) * m_swerveDrive.getMaxAngularSpeed() * speedMultiplier,
            true);
        },
        m_swerveDrive)
    );
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

    //Can Id Motor Test Remove when you are sure you got the CAN ids correct 
    //MAKE SURE YOU DONT GET LEFT AND RIGHT CONFUSED
/*     m_driverController.povUp().whileTrue(
      Commands.run(() -> m_swerveDrive.getFrontLeftDriveMotor().setDesiredState(
          new SwerveModuleState(0.2, new Rotation2d(0))
      ), m_swerveDrive)
    );

    m_driverController.povDown().whileTrue(
      Commands.run(() -> m_swerveDrive.getFrontRightDriveMotor().setDesiredState(
          new SwerveModuleState(0.2, new Rotation2d(0))
      ), m_swerveDrive)
    );

    m_driverController.povLeft().whileTrue(
      Commands.run(() -> m_swerveDrive.getRearLeftDriveMotor().setDesiredState(
          new SwerveModuleState(0.2, new Rotation2d(0))
      ), m_swerveDrive)
    );

    m_driverController.povRight().whileTrue(
      Commands.run(() -> m_swerveDrive.getRearRightDriveMotor().setDesiredState(
          new SwerveModuleState(0.2, new Rotation2d(0))
      ), m_swerveDrive)
    ); */
  }


  private void configureMechanismBindings() {
    m_mechanismController.a().whileTrue(
      new RunCollectorCommand(m_collector, false).alongWith(
        Commands.run(() -> {
          m_shooterSubsystem.runFloor(false);
          m_shooterSubsystem.runIndexer(true);
        }, m_shooterSubsystem)
      )
    );

    m_mechanismController.b().onTrue(
      new StopCollectorCommand(m_collector).alongWith(
        Commands.run(() -> {
          m_shooterSubsystem.StopFloor();
          m_shooterSubsystem.StopIndexer();
        }, m_shooterSubsystem)
      )
    );

    m_mechanismController.x().onTrue(
      new RunCollectorCommand(m_collector, true).alongWith(
        Commands.run(() -> {
          m_shooterSubsystem.runFloor(true);
          m_shooterSubsystem.runIndexer(true);
        }, m_shooterSubsystem)
      )
    );

    m_mechanismController.povUp().onTrue(
      new HopperHalfwaySequenceCommand(m_collector)
    );

    m_mechanismController.povLeft().onTrue(
      new RetractHopperCommand(m_collector)
    );

    m_mechanismController.povRight().onTrue(
      new ExtendHopperCommand(m_collector)
    );

    m_mechanismController.rightTrigger().whileTrue(
      new ShooterWithAutoAimCommand(
        m_shooterSubsystem,
        m_swerveDrive,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * m_swerveDrive.getMaxSpeedMetersPerSecond() * getSpeedMultiplier(),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * m_swerveDrive.getMaxSpeedMetersPerSecond() * getSpeedMultiplier(),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) * m_swerveDrive.getMaxAngularSpeed() * getSpeedMultiplier(),
        () -> m_driverController.leftTrigger().getAsBoolean()
      )
    );

  }

  /*
   * Configures autonomous commands using PathPlanner.
   * Create paths in PathPlanner GUI and place in src/main/deploy/pathplanner/
   *
   * The chooser will appear on the SmartDashboard/Shuffleboard.
   *
   * To add event markers to your path:
   * 1. Open the path in PathPlanner GUI
   * 2. Add event markers (e.g., "startCollector", "shoot", "stopCollector")
   * 3. Register named commands below using NamedCommands.registerCommand()
   */
  private void configureAutos() {
    // Register named commands that can be called from PathPlanner paths
    registerNamedCommands();

    try {
      // Create the ExtendBackupAndShoot autonomous command
      ExtendBackupAndShootCommand extendBackupAndShoot = new ExtendBackupAndShootCommand(
        m_collector,
        m_swerveDrive,
        m_shooterSubsystem
      );

      // Build auto chooser from PathPlanner GUI
      // This will automatically include all autos created in PathPlanner
      m_autoChooser = AutoBuilder.buildAutoChooser();

      // Add custom/fallback options
      m_autoChooser.addOption("ExtendBackupAndShoot", extendBackupAndShoot);
      m_autoChooser.addOption("Do Nothing", Commands.none());

    } catch (Exception e) {
      System.err.println("Failed to load autonomous commands: " + e.getMessage());
      e.printStackTrace();
      m_autoChooser = new SendableChooser<>();
      m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
    }

    // Send the chooser to SmartDashboard (which uses NetworkTables under the hood)
    SmartDashboard.putData("Auto/Chooser", m_autoChooser);
  }

  /**
   * Register named commands that can be used in PathPlanner paths.
   * These commands will be triggered when the path reaches an event marker with the corresponding name.
   */
  private void registerNamedCommands() {
    // Linked to "Blue 1 to collect and shoot" path event markers
    NamedCommands.registerCommand("DeployHopper",
      new ExtendHopperCommand(m_collector));

    NamedCommands.registerCommand("RunCollector",
      new RunCollectorCommand(m_collector, false).alongWith(
        Commands.run(() -> {
          m_shooterSubsystem.runFloor(false);
          m_shooterSubsystem.runIndexer(true);
        }, m_shooterSubsystem)
      ));

    NamedCommands.registerCommand("StopCollector",
      new StopCollectorCommand(m_collector).alongWith(
        Commands.runOnce(() -> {
          m_shooterSubsystem.StopFloor();
          m_shooterSubsystem.StopIndexer();
        }, m_shooterSubsystem)
      ));

    // Additional named commands
    NamedCommands.registerCommand("Shoot",
      new ShooterWithAutoAimCommand(m_shooterSubsystem, m_swerveDrive, null, null, null, null).withTimeout(4.0));
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

  public ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  /**
   * Get the current speed multiplier based on driver controller bumpers
   * @return Speed multiplier (precision, normal, or turbo)
   */
  private double getSpeedMultiplier() {
    if (m_driverController.rightBumper().getAsBoolean()) {
      return OIConstants.kTurboSpeedLimit;
    } else if (m_driverController.leftBumper().getAsBoolean()) {
      return OIConstants.kPrecisionSpeedLimit;
    } else {
      return OIConstants.kNormalSpeedLimit;
    }
  }
}

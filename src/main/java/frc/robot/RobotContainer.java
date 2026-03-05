// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.collector.RunCollectorCommand;
import frc.robot.commands.collector.StopCollectorCommand;
import frc.robot.commands.collector.ManualExtendHopperCommand;
import frc.robot.commands.collector.ManualRetractHopperCommand;
import frc.robot.commands.drive.RotateToTargetCommand;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.commands.led.AprilTagLEDCommand;
import frc.robot.commands.shooter.ShooterSequenceCommand;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.commands.auto.DriveAimShootCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveDriveSubsystem m_swerveDrive = new SwerveDriveSubsystem();
  private final PhotonVisionSubsystem m_visionSubsystem = new PhotonVisionSubsystem(m_swerveDrive);
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final CollectorSubsystem m_collector = new CollectorSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_mechanismController;

  private final Command m_autoCommand;

  public RobotContainer() {
      m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
      m_mechanismController = new CommandXboxController(OperatorConstants.kMechanismControllerPort);

      // Create autonomous command
      m_autoCommand = new DriveAimShootCommand(m_swerveDrive, m_visionSubsystem, m_shooterSubsystem);

      configureDefaultCommands();
      configureDriverBindings();
      configureMechanismBindings();
  }

  private void configureDefaultCommands() {
    m_swerveDrive.setDefaultCommand(
      new SwerveDriveCommand(
        m_swerveDrive,
        () -> -m_driverController.getLeftY(),  
        () -> -m_driverController.getLeftX(),  
        () -> -m_driverController.getRightX(),  
        () -> getSpeedLimit()                   
      )
    );

    if (m_visionSubsystem != null) {
      m_ledSubsystem.setDefaultCommand(
        new AprilTagLEDCommand(m_ledSubsystem, m_visionSubsystem)
      );
    }
  }

  private void configureDriverBindings() {
    m_driverController.start().onTrue(
      m_swerveDrive.runOnce(() -> m_swerveDrive.resetGyro())
    );

    m_driverController.y().onTrue(
      Commands.either(
        new RotateToTargetCommand(m_swerveDrive, AutoConstants.blueTarget),
        new RotateToTargetCommand(m_swerveDrive, AutoConstants.redTarget),
        () -> DriverStation.getAlliance().isPresent() &&
              DriverStation.getAlliance().get() == Alliance.Blue
      )
    );

    // D-pad controls for individual drive motors
    m_driverController.povUp().whileTrue(
      Commands.run(() -> m_swerveDrive.getFrontLeft().setDriveMotorSpeed(0.3), m_swerveDrive)
        .finallyDo(() -> m_swerveDrive.getFrontLeft().setDriveMotorSpeed(0))
    );

    m_driverController.povDown().whileTrue(
      Commands.run(() -> m_swerveDrive.getFrontRight().setDriveMotorSpeed(0.3), m_swerveDrive)
        .finallyDo(() -> m_swerveDrive.getFrontRight().setDriveMotorSpeed(0))
    );

    m_driverController.povLeft().whileTrue(
      Commands.run(() -> m_swerveDrive.getBackLeft().setDriveMotorSpeed(0.3), m_swerveDrive)
        .finallyDo(() -> m_swerveDrive.getBackLeft().setDriveMotorSpeed(0))
    );

    m_driverController.povRight().whileTrue(
      Commands.run(() -> m_swerveDrive.getBackRight().setDriveMotorSpeed(0.3), m_swerveDrive)
        .finallyDo(() -> m_swerveDrive.getBackRight().setDriveMotorSpeed(0))
    );
  }

  private void configureMechanismBindings() {
    m_mechanismController.a().whileTrue(
      new RunCollectorCommand(m_collector, false).alongWith(Commands.run(() ->
        m_shooterSubsystem.runFloor(false), m_shooterSubsystem))
    );

    m_mechanismController.b().onTrue(
      new ParallelCommandGroup(
        new StopCollectorCommand(m_collector),
        Commands.runOnce(() ->
          m_shooterSubsystem.stopAll(), m_shooterSubsystem)
      )
    );

    m_mechanismController.x().onTrue(
      new RunCollectorCommand(m_collector, true)
        .alongWith(Commands.run(() -> m_shooterSubsystem.runFloor(true), m_shooterSubsystem))
    );

    m_mechanismController.rightTrigger().onTrue(
      new ShooterSequenceCommand(m_shooterSubsystem)
    );

    m_mechanismController.povLeft().whileTrue(
      new ManualExtendHopperCommand(m_collector)
    );

    m_mechanismController.povRight().whileTrue(
      new ManualRetractHopperCommand(m_collector)
    );

    m_mechanismController.back().onTrue(
      Commands.runOnce(() -> m_collector.resetHopperEncoder(), m_collector)
    );

    m_mechanismController.povUp().whileTrue(
      Commands.run(() -> m_climber.setSpeed(0.2), m_climber)
        .finallyDo(() -> m_climber.stop())
    );

    m_mechanismController.povDown().whileTrue(
      Commands.run(() -> m_climber.setSpeed(-0.2), m_climber)
        .finallyDo(() -> m_climber.stop())
    );
  }

  public double getSpeedLimit() {
    if (m_driverController.rightBumper().getAsBoolean()) {
      return OperatorConstants.kTurboSpeedLimit;
    } else if (m_driverController.leftBumper().getAsBoolean()) {
      return OperatorConstants.kPrecisionSpeedLimit;
    } else {
      return OperatorConstants.kNormalSpeedLimit;
    }
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  public SwerveDriveSubsystem getSwerveDrive() {
    return m_swerveDrive;
  }

  public PhotonVisionSubsystem getVisionSubsystem() {
    return m_visionSubsystem;
  }

  public LEDSubsystem getLEDSubsystem() {
    return m_ledSubsystem;
  }

  public CommandXboxController getDriverControllerForDebug() {
    return m_driverController;
  }
}

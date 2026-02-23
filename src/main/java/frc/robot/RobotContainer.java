// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.collector.DeployHopperCommand;
import frc.robot.commands.collector.RunCollectorCommand;
import frc.robot.commands.collector.StopCollectorCommand;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.commands.led.AprilTagLEDCommand;
import frc.robot.subsystems.collector.CollectorSubsystem;
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
  private final ShooterSubsystem m_testMotors = new ShooterSubsystem();
  private final CollectorSubsystem m_collector = new CollectorSubsystem();

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_mechanismController;

  //private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
      m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
      m_mechanismController = new CommandXboxController(OperatorConstants.kMechanismControllerPort);

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
  }

  private void configureMechanismBindings() {
    m_mechanismController.a().whileTrue(
      Commands.run(() -> {
        m_testMotors.runAllMotors();
      }, m_testMotors)
      .until(() -> m_testMotors.isAnyMotorOverCurrent(ShooterConstants.kMotorCurrentLimit))
    );

    m_mechanismController.y().onTrue(
      Commands.run(() -> {
        m_testMotors.stopAll();
      }, m_testMotors)
    );

    m_mechanismController.b().onTrue(
     // new DeployHopperCommand(m_collector).andThen(
        new RunCollectorCommand(m_collector)
     // )
    );

    m_mechanismController.x().onTrue(
      new StopCollectorCommand(m_collector)
    );

    m_mechanismController.povUp().onTrue(
      Commands.run(() -> {
        ShooterSubsystem.setKTargetRPM(getSpeedLimit() + 1000);
      })
    );

    m_mechanismController.povDown().onTrue(
      Commands.run(() -> {
        ShooterSubsystem.setKTargetRPM(getSpeedLimit() - 1000);
      })
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

/*
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
*/

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.commands.led.AprilTagLEDCommand;
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

    m_driverController.y().onTrue(
      Commands.either(
        m_swerveDrive.rotateToTarget(AutoConstants.kRedTargetX, AutoConstants.kRedTargetY),
        m_swerveDrive.rotateToTarget(AutoConstants.kBlueTargetX, AutoConstants.kBlueTargetY),
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        }
      ).andThen(
        // Hold X-formation until driver gives any stick input
        m_swerveDrive.run(() -> m_swerveDrive.setX())
          .until(() ->
            Math.abs(m_driverController.getLeftY()) > 0.1 ||
            Math.abs(m_driverController.getLeftX()) > 0.1 ||
            Math.abs(m_driverController.getRightX()) > 0.1
          )
      )
    );
  }

  private void configureMechanismBindings() {
    // A button: Run shooter with distance-based RPM
    m_mechanismController.a().whileTrue(
      Commands.run(() -> {
        // Get distance to speaker from vision
        double distance = m_visionSubsystem.getDistanceToSpeaker();

        // Calculate RPM based on distance
        double targetRPM = ShooterConstants.getRPMForDistance(distance);

        // Run motors at calculated RPM
        m_testMotors.runAllMotors(targetRPM);
      }, m_testMotors)
      .until(() -> m_testMotors.isAnyMotorOverCurrent(ShooterConstants.kMotorCurrentLimit))
    );

    // Y button: Stop all shooter motors
    m_mechanismController.y().onTrue(
      Commands.run(() -> {
        m_testMotors.stopAll();
      }, m_testMotors)
    );
  }

  private double getSpeedLimit() {
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

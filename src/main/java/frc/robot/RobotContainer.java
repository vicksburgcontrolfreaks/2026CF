// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotContainerConstants;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.commands.led.AprilTagLEDCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.auto.DriveForwardCommand;
import frc.robot.commands.collector.RunCollector;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.subsystems.shooter.ShooterAdjustments;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private boolean m_collectorHalfSpeed = false;

  private final CollectorSubsystem m_collector = null;
  private final SwerveDriveSubsystem m_swerveDrive = new SwerveDriveSubsystem();
  private final PhotonVisionSubsystem m_visionSubsystem = new PhotonVisionSubsystem(m_swerveDrive);
  private final ShooterAdjustments m_shooter = null;
  private final ClimberSubsystem m_climber = null;
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_mechanismController;
  private final JoystickController m_joystickController;

  private final SendableChooser<Command> m_autoChooser;

  private boolean m_pathPlannerConfigured = false;

  public RobotContainer() {
    System.out.println(">>> RobotContainer constructor started <<<");

    if (RobotContainerConstants.kUseJoystick) {
      System.out.println(">>> Using Logitech Extreme 3D Pro Joystick <<<");
      m_driverController = null;
      m_mechanismController = null;
      m_joystickController = new JoystickController(m_swerveDrive, m_collector);
    } else {
      System.out.println(">>> Using Dual Xbox Controllers <<<");
      m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
      m_mechanismController = new CommandXboxController(OperatorConstants.kMechanismControllerPort);
      m_joystickController = null;

      configureDefaultCommands();

      configureDriverBindings();
      configureMechanismBindings();
    }

    m_pathPlannerConfigured = configurePathPlanner();

    System.out.println(">>> RobotContainer constructor finished - Button bindings configured! <<<");

    if (m_pathPlannerConfigured) {
      m_autoChooser = AutoBuilder.buildAutoChooser();
      m_autoChooser.addOption("Drive Forward 1 Meter", new DriveForwardCommand(m_swerveDrive, 1.0));
      m_autoChooser.addOption("Drive Forward 2 Meters", new DriveForwardCommand(m_swerveDrive, 2.0));
      SmartDashboard.putData("Auto Chooser", m_autoChooser);
    } else {
      m_autoChooser = new SendableChooser<>();
      m_autoChooser.setDefaultOption("Drive Forward 1 Meter", new DriveForwardCommand(m_swerveDrive, 1.0));
      m_autoChooser.addOption("Drive Forward 2 Meters", new DriveForwardCommand(m_swerveDrive, 2.0));
      m_autoChooser.addOption("Do Nothing", Commands.none());
      SmartDashboard.putData("Auto Chooser", m_autoChooser);
      DriverStation.reportWarning("PathPlanner configuration failed! Using simple autonomous options.", false);
    }
  }

  private boolean configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
        m_swerveDrive::getPose,
        m_swerveDrive::resetOdometry,
        m_swerveDrive::getChassisSpeeds,
        (speeds, feedforwards) -> m_swerveDrive.setChassisSpeeds(speeds),
        new PPHolonomicDriveController(
          new PIDConstants(AutoConstants.kPTranslation, AutoConstants.kITranslation, AutoConstants.kDTranslation),
          new PIDConstants(AutoConstants.kPRotation, AutoConstants.kIRotation, AutoConstants.kDRotation)
        ),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        m_swerveDrive
      );

      NamedCommands.registerCommand("stopDrive", m_swerveDrive.runOnce(() -> m_swerveDrive.stop()));
      NamedCommands.registerCommand("lockWheels", m_swerveDrive.run(() -> m_swerveDrive.setX()).withTimeout(RobotContainerConstants.kSetXTimeoutSeconds));

      return true;
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner RobotConfig! Please open PathPlanner GUI and configure your robot.", false);
      e.printStackTrace();
      return false;
    }
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

    m_driverController.back().onTrue(
      m_swerveDrive.runOnce(() -> m_swerveDrive.toggleFieldOriented())
    );

    if (m_collector != null) {
      m_driverController.x().onTrue(
         Commands.runOnce(() -> m_collectorHalfSpeed = !m_collectorHalfSpeed)
      );

      m_driverController.a().whileTrue(
         Commands.run(() -> {
          double speed = m_collectorHalfSpeed ? RobotContainerConstants.kCollectorHalfSpeed : RobotContainerConstants.kCollectorFullSpeed;
         m_collector.run(speed);
        }, m_collector)
      );

      m_driverController.b().whileTrue(
         Commands.run(() -> {
          double speed = m_collectorHalfSpeed ? RobotContainerConstants.kCollectorHalfSpeedReverse : RobotContainerConstants.kCollectorFullSpeedReverse;
         m_collector.run(speed);
        }, m_collector)
      );
    }

    if (m_climber != null) {
      m_driverController.povUp().whileTrue(
        Commands.run(() -> m_climber.extend(), m_climber)
      );

      m_driverController.povDown().whileTrue(
        Commands.run(() -> m_climber.retract(), m_climber)
      );
    }

    if (m_shooter != null) {
      m_driverController.y().whileTrue(
        new ShootCommand(m_shooter)
      );
    }

    m_driverController.pov(0).onTrue(
      m_swerveDrive.rotateToAngle(0)
    );

    m_driverController.pov(90).onTrue(
      m_swerveDrive.rotateToAngle(-90)
    );

    m_driverController.pov(180).onTrue(
      m_swerveDrive.rotateToAngle(180)
    );

    m_driverController.pov(270).onTrue(
      m_swerveDrive.rotateToAngle(90)
    );

    m_driverController.y().onTrue(
      Commands.either(
        m_swerveDrive.rotateToTarget(AutoConstants.kRedTargetX, AutoConstants.kRedTargetY),
        m_swerveDrive.rotateToTarget(AutoConstants.kBlueTargetX, AutoConstants.kBlueTargetY),
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        }
      )
    );
  }

  private void configureMechanismBindings() {
    if (m_shooter != null) {
      m_mechanismController.a().whileTrue(
        new ShootCommand(m_shooter)
      );
    }

    if (m_climber != null) {
      m_mechanismController.rightBumper().whileTrue(
        Commands.run(() -> m_climber.extend(), m_climber)
      );

      m_mechanismController.leftBumper().whileTrue(
        Commands.run(() -> m_climber.retract(), m_climber)
      );
    }

    if (m_collector != null) {
      m_mechanismController.b().onTrue(
         Commands.runOnce(() -> m_collector.deploy(), m_collector)
      );

      m_mechanismController.x().onTrue(
         Commands.runOnce(() -> m_collector.retract(), m_collector)
      );

      m_mechanismController.y().onTrue(
         Commands.runOnce(() -> m_collector.toggle(), m_collector)
      );
    }
  }

  private double getSpeedLimit() {
    double rightTrigger = m_driverController.getRightTriggerAxis();
    double leftTrigger = m_driverController.getLeftTriggerAxis();

    if (rightTrigger > RobotContainerConstants.kTriggerThreshold) {
      return OperatorConstants.kTurboSpeedLimit;
    } else if (leftTrigger > RobotContainerConstants.kTriggerThreshold) {
      return OperatorConstants.kPrecisionSpeedLimit;
    } else {
      return OperatorConstants.kNormalSpeedLimit;
    }
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
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

  public JoystickController getJoystickContainerForDebug() {
    return m_joystickController;
  }

  public boolean isUsingJoystick() {
    return RobotContainerConstants.kUseJoystick;
  }
}

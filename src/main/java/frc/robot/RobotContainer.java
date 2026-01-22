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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.subsystems.shooter.ShooterAdjustments;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final PhotonVisionSubsystem m_visionSubsystem = new PhotonVisionSubsystem(m_swerveDrive);
  private final ShooterAdjustments m_shooter = new ShooterAdjustments();

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Autonomous chooser
  private final SendableChooser<Command> m_autoChooser;

  // PathPlanner configuration status
  private boolean m_pathPlannerConfigured = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure PathPlanner AutoBuilder
    m_pathPlannerConfigured = configurePathPlanner();

    // Configure the default command for swerve drive
    configureDefaultCommands();

    // Configure the trigger bindings
    configureBindings();

    // Build autonomous chooser from PathPlanner only if configured successfully
    if (m_pathPlannerConfigured) {
      m_autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", m_autoChooser);
    } else {
      // Create a disabled auto chooser if PathPlanner failed
      m_autoChooser = new SendableChooser<>();
      m_autoChooser.setDefaultOption("Auto Disabled - PathPlanner Config Failed", Commands.none());
      SmartDashboard.putData("Auto Chooser", m_autoChooser);

      DriverStation.reportWarning("PathPlanner configuration failed! Autonomous disabled.", false);
    }
  }

  /**
   * Configure PathPlanner AutoBuilder for autonomous path following
   * @return true if configuration succeeded, false otherwise
   */
  private boolean configurePathPlanner() {
    try {
      // Load the RobotConfig from the GUI-generated file in deploy/pathplanner/
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
        m_swerveDrive::getPose, // Robot pose supplier
        m_swerveDrive::resetOdometry, // Method to reset odometry
        m_swerveDrive::getChassisSpeeds, // ChassisSpeeds supplier (robot relative)
        (speeds, feedforwards) -> m_swerveDrive.setChassisSpeeds(speeds), // Method to drive robot
        new PPHolonomicDriveController(
          new PIDConstants(AutoConstants.kPTranslation, AutoConstants.kITranslation, AutoConstants.kDTranslation), // Translation PID
          new PIDConstants(AutoConstants.kPRotation, AutoConstants.kIRotation, AutoConstants.kDRotation) // Rotation PID
        ),
        config, // Robot config
        () -> {
          // Flip path based on alliance color (Red alliance needs flipped paths)
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        m_swerveDrive // Drive subsystem
      );

      // Register named commands for PathPlanner
      // These can be used in the PathPlanner GUI as event markers
      NamedCommands.registerCommand("stopDrive", m_swerveDrive.runOnce(() -> m_swerveDrive.stop()));
      NamedCommands.registerCommand("lockWheels", m_swerveDrive.run(() -> m_swerveDrive.setX()).withTimeout(1.0));

      return true;
    } catch (Exception e) {
      // If RobotConfig file doesn't exist, PathPlanner will not work properly
      // You need to open PathPlanner GUI and configure your robot, which will generate the config file
      DriverStation.reportError("Failed to load PathPlanner RobotConfig! Please open PathPlanner GUI and configure your robot.", false);
      e.printStackTrace();
      return false;
    }
  }

  private void configureDefaultCommands() {
    // Set default drive command with normal speed
    m_swerveDrive.setDefaultCommand(
      new SwerveDriveCommand(
        m_swerveDrive,
        () -> -m_driverController.getLeftY(),   // Forward/backward (inverted)
        () -> -m_driverController.getLeftX(),   // Left/right (inverted)
        () -> -m_driverController.getRightX(),  // Rotation (inverted)
        () -> getSpeedLimit()                   // Speed limit based on triggers
      )
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Reset gyro to zero
    m_driverController.start().onTrue(
      m_swerveDrive.runOnce(() -> m_swerveDrive.resetGyro())
    );

    // Toggle field-oriented drive
    m_driverController.back().onTrue(
      m_swerveDrive.runOnce(() -> m_swerveDrive.toggleFieldOriented())
    );

    // X-configuration for defense (locks wheels)
    m_driverController.x().whileTrue(
      m_swerveDrive.run(() -> m_swerveDrive.setX())
    );

    // Reset odometry to origin
    m_driverController.y().onTrue(
      m_swerveDrive.runOnce(() -> m_swerveDrive.resetOdometry(new edu.wpi.first.math.geometry.Pose2d()))
    );

    // Run shooter while A button is held
    m_driverController.a().whileTrue(
      new ShootCommand(m_shooter)
    );
  }

  /**
   * Gets the current speed limit based on trigger inputs
   * Right Trigger = Turbo (100%)
   * Left Trigger = Precision (30%)
   * No Trigger = Normal (80%)
   */
  private double getSpeedLimit() {
    double rightTrigger = m_driverController.getRightTriggerAxis();
    double leftTrigger = m_driverController.getLeftTriggerAxis();

    if (rightTrigger > 0.1) {
      return OperatorConstants.kTurboSpeedLimit;
    } else if (leftTrigger > 0.1) {
      return OperatorConstants.kPrecisionSpeedLimit;
    } else {
      return OperatorConstants.kNormalSpeedLimit;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return the selected autonomous command from the chooser
    // The chooser is populated with all PathPlanner .auto files
    // Selection is made via SmartDashboard/Elastic dashboard
    return m_autoChooser.getSelected();
  }

  /**
   * Get the swerve drive subsystem
   */
  public SwerveDrive getSwerveDrive() {
    return m_swerveDrive;
  }

  /**
   * Get the vision subsystem
   */
  public PhotonVisionSubsystem getVisionSubsystem() {
    return m_visionSubsystem;
  }
}

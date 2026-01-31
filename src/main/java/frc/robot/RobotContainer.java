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
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.subsystems.shooter.ShooterAdjustments;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.RunCollector;
import frc.robot.subsystems.climber.Climber;
import frc.robot.commands.climber.MoveClimberCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private boolean m_collectorHalfSpeed = false;
  private double m_collectorSpeed = RobotContainerConstants.kCollectorFullSpeed;

  // TEMPORARILY DISABLED: Collector subsystem (Spark Max ID 11 timeout issue)
  // TODO: Re-enable when collector motors are connected
  // private final Collector m_collector = new Collector();
  private final Collector m_collector = null;

  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  // TEMPORARILY DISABLED: PhotonVision subsystem (no coprocessor detected on network)
  // TODO: Re-enable when PhotonVision coprocessor is connected and configured
  // private final PhotonVisionSubsystem m_visionSubsystem = new PhotonVisionSubsystem(m_swerveDrive);
  private final PhotonVisionSubsystem m_visionSubsystem = null;

  private final ShooterAdjustments m_shooter = new ShooterAdjustments(m_swerveDrive);

  private final Climber m_climber = new Climber();

  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  // Controllers - only one will be initialized based on USE_JOYSTICK flag
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_mechanismController;
  private final JoystickContainer m_joystickContainer;

  // Autonomous chooser
  private final SendableChooser<Command> m_autoChooser;

  // PathPlanner configuration status
  private boolean m_pathPlannerConfigured = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    System.out.println(">>> RobotContainer constructor started <<<");

    // Initialize the appropriate controller based on kUseJoystick flag
    if (RobotContainerConstants.kUseJoystick) {
      System.out.println(">>> Using Logitech Extreme 3D Pro Joystick <<<");
      m_driverController = null;
      m_mechanismController = null;
      m_joystickContainer = new JoystickContainer(m_swerveDrive, m_collector);
    } else {
      System.out.println(">>> Using Dual Xbox Controllers <<<");
      m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
      m_mechanismController = new CommandXboxController(OperatorConstants.kMechanismControllerPort);
      m_joystickContainer = null;

      // Configure the default command for swerve drive (Xbox only)
      configureDefaultCommands();

      // Configure the trigger bindings (Xbox only)
      configureDriverBindings();
      configureMechanismBindings();
    }

    // Configure PathPlanner AutoBuilder
    m_pathPlannerConfigured = configurePathPlanner();

    System.out.println(">>> RobotContainer constructor finished - Button bindings configured! <<<");

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
      NamedCommands.registerCommand("lockWheels", m_swerveDrive.run(() -> m_swerveDrive.setX()).withTimeout(RobotContainerConstants.kSetXTimeoutSeconds));

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
   * Configure button bindings for the driver controller.
   * Driver controls: robot movement, rotation, gyro reset, field-oriented toggle
   */
  private void configureDriverBindings() {
    // Reset gyro to zero
    m_driverController.start().onTrue(
      m_swerveDrive.runOnce(() -> m_swerveDrive.resetGyro())
    );

    // Toggle field-oriented drive
    m_driverController.back().onTrue(
      m_swerveDrive.runOnce(() -> m_swerveDrive.toggleFieldOriented())
    );

    // Rotate to cardinal directions using D-pad (POV)
    // POV up (0°) -> Rotate to 0° (forward)
    m_driverController.pov(0).onTrue(
      m_swerveDrive.rotateToAngle(0)
    );

    // POV right (90°) -> Rotate to -90° (right)
    m_driverController.pov(90).onTrue(
      m_swerveDrive.rotateToAngle(-90)
    );

    // POV down (180°) -> Rotate to 180° (backward)
    m_driverController.pov(180).onTrue(
      m_swerveDrive.rotateToAngle(180)
    );

    // POV left (270°) -> Rotate to 90° (left)
    m_driverController.pov(270).onTrue(
      m_swerveDrive.rotateToAngle(90)
    );

    // Y button - Rotate to face targets based on robot position
    // If X < 5.2: rotate to red target
    // If X > 12.5: rotate to blue target
    // Otherwise: do nothing
    m_driverController.y().onTrue(
      Commands.either(
        // If X < 5.2, rotate to red target
        m_swerveDrive.rotateToTarget(AutoConstants.kRedTargetX, AutoConstants.kRedTargetY),
        // Otherwise, check if X > 12.5 for blue target
        Commands.either(
          m_swerveDrive.rotateToTarget(AutoConstants.kBlueTargetX, AutoConstants.kBlueTargetY),
          Commands.none(),
          () -> m_swerveDrive.getPose().getX() > AutoConstants.kBlueTargetX
        ),
        // Condition: check if robot X < 5.2
        () -> m_swerveDrive.getPose().getX() < AutoConstants.kRedTargetX
      )
    );
  }

  /**
   * Configure button bindings for the mechanism controller.
   * Mechanism controls: shooter, climber, collector
   */
  private void configureMechanismBindings() {
    // Shooter control
    // A button - Run shooter (distance-based power)
    m_mechanismController.a().whileTrue(
      new ShootCommand(m_shooter)
    );

    // Climber controls
    // Right bumper - Climb up
    m_mechanismController.rightBumper().whileTrue(
      Commands.run(() -> m_climber.climbUp(), m_climber)
    );

    // Left bumper - Climb down
    m_mechanismController.leftBumper().whileTrue(
      Commands.run(() -> m_climber.climbDown(), m_climber)
    );

    // Collector controls - only bind if collector is available
    if (m_collector != null) {
      // B button - Deploy collector (extends and starts motors automatically)
      m_mechanismController.b().onTrue(
         Commands.runOnce(() -> m_collector.deploy(), m_collector)
      );

      // X button - Retract collector (retracts and stops motors automatically)
      m_mechanismController.x().onTrue(
         Commands.runOnce(() -> m_collector.retract(), m_collector)
      );

      // Y button - Toggle collector deployment
      m_mechanismController.y().onTrue(
         Commands.runOnce(() -> m_collector.toggle(), m_collector)
      );
    }
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

    if (rightTrigger > RobotContainerConstants.kTriggerThreshold) {
      return OperatorConstants.kTurboSpeedLimit;
    } else if (leftTrigger > RobotContainerConstants.kTriggerThreshold) {
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
   * Returns null if PhotonVision is temporarily disabled
   */
  public PhotonVisionSubsystem getVisionSubsystem() {
    return m_visionSubsystem;
  }

  /**
   * Get the LED subsystem
   */
  public LEDSubsystem getLEDSubsystem() {
    return m_ledSubsystem;
  }

  /**
   * Get the driver controller (for debugging)
   * Returns null if joystick is being used
   */
  public CommandXboxController getDriverControllerForDebug() {
    return m_driverController;
  }

  /**
   * Get the joystick container (for debugging)
   * Returns null if Xbox controller is being used
   */
  public JoystickContainer getJoystickContainerForDebug() {
    return m_joystickContainer;
  }

  /**
   * Check if joystick mode is active
   */
  public boolean isUsingJoystick() {
    return RobotContainerConstants.kUseJoystick;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveDrive);

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the default command for swerve drive
    configureDefaultCommands();

    // Configure the trigger bindings
    configureBindings();
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
    // This will be replaced with PathPlanner autos later
    return m_swerveDrive.run(() -> m_swerveDrive.stop());
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
  public VisionSubsystem getVisionSubsystem() {
    return m_visionSubsystem;
  }
}

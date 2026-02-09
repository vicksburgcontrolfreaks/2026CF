// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotContainerConstants;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Alternative controller configuration using Logitech Extreme 3D Pro joystick.
 * This class mirrors the functionality of the Xbox controller setup but uses
 * a single joystick with twist axis for rotation.
 *
 * Button mapping for Extreme 3D Pro:
 * - Stick X/Y: Strafe and Forward/Backward (translation)
 * - Stick Z-axis (twist): Rotation
 * - Throttle Slider: Available for speed control (currently using buttons)
 * - Button 1 (Trigger): Turbo mode
 * - Button 2: Precision mode
 * - Button 3: Reset gyro
 * - Button 4: Toggle field-oriented drive
 * - Button 5: Toggle collector half-speed
 * - Button 6: Run collector forward
 * - Button 7: Run collector reverse
 * - Buttons 8-12: Available for future use
 */
public class JoystickContainer {

    private final CommandJoystick m_joystick;
    private final SwerveDrive m_swerveDrive;
    private final Collector m_collector;

    private boolean m_collectorHalfSpeed = false;
    private boolean m_turboMode = false;
    private boolean m_precisionMode = false;

    /**
     * Creates a new JoystickContainer with Extreme 3D Pro configuration
     * @param swerveDrive The swerve drive subsystem (required)
     * @param collector The collector subsystem (can be null if disabled)
     */
    public JoystickContainer(SwerveDrive swerveDrive, Collector collector) {
        if (swerveDrive == null) {
            throw new IllegalArgumentException("SwerveDrive cannot be null");
        }

        this.m_swerveDrive = swerveDrive;
        this.m_collector = collector;
        this.m_joystick = new CommandJoystick(OperatorConstants.kJoystickPort);

        configureDefaultCommands();
        configureBindings();

        // Log warning if collector is disabled
        if (collector == null) {
            System.out.println("JoystickContainer: Collector is disabled - collector buttons will not be bound");
        }
    }

    /**
     * Configure default commands for joystick control
     */
    private void configureDefaultCommands() {
        // Set default drive command
        // Extreme 3D Pro axes: X=strafe (left/right), Y=forward/back, Z=twist (rotation)
        m_swerveDrive.setDefaultCommand(
            new SwerveDriveCommand(
                m_swerveDrive,
                () -> -m_joystick.getY(),      // Forward/backward (inverted)
                () -> -m_joystick.getX(),      // Left/right strafe (inverted)
                () -> -m_joystick.getZ(),      // Rotation via Z-axis/twist (inverted)
                () -> getSpeedLimit()          // Speed limit based on buttons/throttle
            )
        );
    }

    /**
     * Configure button bindings for the joystick
     */
    private void configureBindings() {
        // Button 1 (Trigger on stick) - Toggle Turbo mode
        m_joystick.button(1).onTrue(
            Commands.runOnce(() -> {
                m_turboMode = !m_turboMode;
                if (m_turboMode) m_precisionMode = false; // Disable precision when turbo enabled
            })
        );

        // Button 2 - Toggle Precision mode
        m_joystick.button(2).onTrue(
            Commands.runOnce(() -> {
                m_precisionMode = !m_precisionMode;
                if (m_precisionMode) m_turboMode = false; // Disable turbo when precision enabled
            })
        );

        // Button 3 - Reset gyro to zero
        m_joystick.button(3).onTrue(
           // m_swerveDrive.runOnce(() -> m_swerveDrive.resetGyro())
           m_swerveDrive.runOnce(() -> m_swerveDrive.toggleFieldOriented())
        );

        // Button 4 - Toggle field-oriented drive
        m_joystick.button(4).onTrue(
            m_swerveDrive.runOnce(() -> m_swerveDrive.toggleFieldOriented())
        );

        // Collector controls - only bind if collector is available
        if (m_collector != null) {
            // Button 5 - Toggle collector half-speed
            m_joystick.button(5).onTrue(
                Commands.runOnce(() -> m_collectorHalfSpeed = !m_collectorHalfSpeed)
            );

            // Button 6 - Run collector forward (while held)
            m_joystick.button(6).whileTrue(
                Commands.run(() -> {
                    double speed = m_collectorHalfSpeed ? 0.5 : 1.0;
                    m_collector.run(speed);
                }, m_collector)
            );

            // Button 7 - Run collector reverse (while held)
            m_joystick.button(7).whileTrue(
                Commands.run(() -> {
                    double speed = m_collectorHalfSpeed ? -0.5 : -1.0;
                    m_collector.run(speed);
                }, m_collector)
            );
        }

        // Buttons 8-12 (or 5-12 if collector disabled) available for future functionality
    }

    /**
     * Gets the current speed limit based on button toggles
     * Button 1 (Trigger) = Turbo (100%)
     * Button 2 = Precision (30%)
     * Neither = Normal (80%)
     *
     * Note: You could also use the throttle slider by reading m_joystick.getThrottle()
     * and mapping it from [-1, 1] to your desired speed range
     */
    private double getSpeedLimit() {
        if (m_turboMode) {
            return OperatorConstants.kTurboSpeedLimit;
        } else if (m_precisionMode) {
            return OperatorConstants.kPrecisionSpeedLimit;
        } else {
            return OperatorConstants.kNormalSpeedLimit;
        }
    }

    /**
     * Alternative speed limit method using throttle slider
     * Uncomment and use this in SwerveDriveCommand if you prefer continuous throttle control
     *
     * Throttle ranges from -1 (back) to +1 (forward)
     * This maps it to: back=-1 → min speed, forward=+1 → max speed
     */
    @SuppressWarnings("unused")
    private double getSpeedLimitFromThrottle() {
        double throttle = m_joystick.getThrottle(); // Range: -1 to +1

        // Map throttle to speed range
        // -1 (back) → precision speed
        // 0 (middle) → normal speed
        // +1 (forward) → turbo speed

        if (throttle < RobotContainerConstants.kThrottleLowerThreshold) {
            // Back position - precision mode
            return OperatorConstants.kPrecisionSpeedLimit;
        } else if (throttle > RobotContainerConstants.kThrottleUpperThreshold) {
            // Forward position - turbo mode
            return OperatorConstants.kTurboSpeedLimit;
        } else {
            // Middle position - normal mode
            return OperatorConstants.kNormalSpeedLimit;
        }
    }

    /**
     * Get the joystick (for debugging purposes)
     */
    public CommandJoystick getJoystick() {
        return m_joystick;
    }
}

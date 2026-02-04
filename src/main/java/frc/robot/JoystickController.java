// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Alternative controller configuration using Logitech Extreme 3D Pro joystick.
 *
 * Button mapping:
 * - Button 1 (Trigger): Toggle turbo mode
 * - Button 2: Toggle precision mode
 * - Button 3: Reset gyro
 * - Button 4: Toggle field-oriented drive
 * - Button 5: Toggle collector half-speed
 * - Button 6: Run collector forward
 * - Button 7: Run collector reverse
 */
public class JoystickController {

    private final CommandJoystick m_joystick;
    private final SwerveDriveSubsystem m_swerveDrive;
    private final CollectorSubsystem m_collector;

    private boolean m_collectorHalfSpeed = false;
    private boolean m_turboMode = false;
    private boolean m_precisionMode = false;

    public JoystickController(SwerveDriveSubsystem swerveDrive, CollectorSubsystem collector) {
        this.m_swerveDrive = swerveDrive;
        this.m_collector = collector;
        this.m_joystick = new CommandJoystick(OperatorConstants.kJoystickPort);

        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
        m_swerveDrive.setDefaultCommand(
            new SwerveDriveCommand(
                m_swerveDrive,
                () -> -m_joystick.getY(),
                () -> -m_joystick.getX(),
                () -> -m_joystick.getZ(),
                () -> getSpeedLimit()
            )
        );
    }

    private void configureBindings() {
        m_joystick.button(1).onTrue(
            Commands.runOnce(() -> {
                m_turboMode = !m_turboMode;
                if (m_turboMode) m_precisionMode = false;
            })
        );

        m_joystick.button(2).onTrue(
            Commands.runOnce(() -> {
                m_precisionMode = !m_precisionMode;
                if (m_precisionMode) m_turboMode = false;
            })
        );

        m_joystick.button(3).onTrue(
            m_swerveDrive.runOnce(() -> m_swerveDrive.resetGyro())
        );

        m_joystick.button(4).onTrue(
            m_swerveDrive.runOnce(() -> m_swerveDrive.toggleFieldOriented())
        );

        if (m_collector != null) {
            m_joystick.button(5).onTrue(
                Commands.runOnce(() -> m_collectorHalfSpeed = !m_collectorHalfSpeed)
            );

            m_joystick.button(6).whileTrue(
                Commands.run(() -> {
                    double speed = m_collectorHalfSpeed ? 0.5 : 1.0;
                    m_collector.run(speed);
                }, m_collector)
            );

            m_joystick.button(7).whileTrue(
                Commands.run(() -> {
                    double speed = m_collectorHalfSpeed ? -0.5 : -1.0;
                    m_collector.run(speed);
                }, m_collector)
            );
        }
    }

    private double getSpeedLimit() {
        if (m_turboMode) {
            return OperatorConstants.kTurboSpeedLimit;
        } else if (m_precisionMode) {
            return OperatorConstants.kPrecisionSpeedLimit;
        } else {
            return OperatorConstants.kNormalSpeedLimit;
        }
    }

    public CommandJoystick getJoystick() {
        return m_joystick;
    }
}

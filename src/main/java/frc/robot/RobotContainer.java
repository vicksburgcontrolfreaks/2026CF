// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.DriveAndShootNoCameraCommand;
import frc.robot.commands.auton.BlueCenterShootCommand;
import frc.robot.commands.auton.BlueLeftCollectAndShootCommand;
import frc.robot.commands.auton.BlueRightCollectAndShootCommand;
import frc.robot.commands.auton.RedCenterShootCommand;
import frc.robot.commands.auton.RedLeftCollectAndShootCommand;
import frc.robot.commands.auton.RedRightCollectAndShootCommand;
import frc.robot.commands.auton.RedRightOverHumpCommand;
import frc.robot.commands.auton.WaypointPIDTestCommand;
import frc.robot.commands.collector.RunCollectorCommand;
import frc.robot.commands.collector.StopCollectorCommand;
import frc.robot.commands.collector.ExtendHopperCommand;
import frc.robot.commands.collector.RetractHopperCommand;
import frc.robot.commands.collector.HopperPopCommand;
import frc.robot.commands.drive.RotateToTargetCommand;
import frc.robot.commands.led.AprilTagLEDCommand;
import frc.robot.commands.shooter.ShooterWithAutoAimCommand;
import frc.robot.commands.test.ShooterTestCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.utils.ShooterTestLogger;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.DoubleEntry;


public class RobotContainer {
  private final DriveSubsystem m_swerveDrive = new DriveSubsystem();
  private final PhotonVisionSubsystem m_visionSubsystem = new PhotonVisionSubsystem(m_swerveDrive);
  private final LEDSubsystem m_ledSubsystem = null;
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(m_visionSubsystem, m_swerveDrive);
  private final CollectorSubsystem m_collector = new CollectorSubsystem();

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_mechanismController;

  private SendableChooser<Command> m_autoChooser;

  // Shooter test system
  private final ShooterTestLogger m_shooterTestLogger = new ShooterTestLogger();
  private ShooterTestCommand m_currentTestCommand = null;
  private IntegerEntry m_testBallsScoredEntry;
  private IntegerEntry m_testBallsLoadedEntry;
  private DoubleEntry m_testDurationEntry;
  private DoubleEntry m_testShooterRPMEntry;
  private DoubleEntry m_testIndexerRPMEntry;
  private DoubleEntry m_testMeasuredDistanceEntry;

  // Alignment PID tuning (public for ShooterCommand access)
  public DoubleEntry m_alignmentPEntry;
  public DoubleEntry m_alignmentIEntry;
  public DoubleEntry m_alignmentDEntry;
  public DoubleEntry m_alignmentToleranceEntry;

  // NOTE: Shooter/Indexer PID tuning moved to ShooterDashboard for better organization

  public RobotContainer() {
      m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
      m_mechanismController = new CommandXboxController(OIConstants.kMechanismControllerPort);

      // BEST PRACTICE: Initialize dashboard manager early so Shuffleboard tabs are created
      // before subsystem initialization completes
      frc.robot.dashboard.DashboardManager.getInstance().initialize();

      configureShooterTestSystem();
      configureAlignmentPIDTuning();
      // configureMotorPIDTuning();  // REMOVED - now handled by ShooterDashboard
      configureAutos();

      // Connect shooter subsystem to this container for PID tuning access
      // m_shooterSubsystem.setContainer(this);  // REMOVED - no longer needed with dashboard

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
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kRotationDeadband) * m_swerveDrive.getMaxAngularSpeed() * speedMultiplier,
            true);
        },
        m_swerveDrive)
    );

    if (m_visionSubsystem != null && m_ledSubsystem != null) {
      m_ledSubsystem.setDefaultCommand(
        new AprilTagLEDCommand(m_ledSubsystem, m_visionSubsystem)
      );
    }
  }

  private void configureDriverBindings() {
    m_driverController.start().onTrue(
      m_swerveDrive.runOnce(() -> m_swerveDrive.zeroHeading())
    );


    m_driverController.x()
      .and(() -> m_swerveDrive.isAlignedToSpeaker())
      .whileTrue(
        m_swerveDrive.runOnce(() -> m_swerveDrive.setX()).andThen(Commands.idle(m_swerveDrive))
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
    // A Button: Run collector in collection mode (forward) - stays on until X or B pressed
    m_mechanismController.a().onTrue(
      Commands.runOnce(() -> {
        m_collector.setHopperPosition(0.18);
        m_collector.runCollector(false);
        m_shooterSubsystem.runFloorSlow(true);  // Run live bottom slowly in reverse during collection
        m_shooterSubsystem.runIndexer(true);
      }, m_collector, m_shooterSubsystem)
    );

    // X Button: Reverse collector (unjam/eject) - stops collector started by A, runs in reverse while held
    m_mechanismController.x().whileTrue(
      new RunCollectorCommand(m_collector, true).alongWith(Commands.run(() -> {
        m_shooterSubsystem.runFloor(true);
        m_shooterSubsystem.runIndexer(true);
      }, m_shooterSubsystem))
    );

    // B Button: Stop all mechanisms EXCEPT shooter (collector, floor, indexer)
    m_mechanismController.b().onTrue(
      new StopCollectorCommand(m_collector).alongWith(
        Commands.runOnce(() -> {
          m_shooterSubsystem.StopFloor();
          m_shooterSubsystem.StopIndexer();
          // Note: Shooter continues spinning (stays ready for quick shots)
        }, m_shooterSubsystem)
      )
    );

    // Y Button: Run floor motor only (for testing/manual feeding)
    m_mechanismController.y().whileTrue(
      Commands.run(() -> {
        m_shooterSubsystem.runFloor(false);  // Run floor motor forward
      }, m_shooterSubsystem)
      .finallyDo(() -> {
        m_shooterSubsystem.StopFloor();
      })
    );

    m_mechanismController.povUp().onTrue(
      new RetractHopperCommand(m_collector)
    );

    m_mechanismController.povDown().onTrue(
      new ExtendHopperCommand(m_collector)
    );

    // Left Bumper: Hopper pop command - alternates hopper up/down every 0.25s while held
    m_mechanismController.leftBumper().whileTrue(
      new HopperPopCommand(m_collector)
    );

    // Manual shoot — shooter already spinning at 3500, just feed immediately, no alignment/vision needed
    Timer manualShootHopperTimer = new Timer();
    boolean[] manualHopperPopHigh = {false};
    m_mechanismController.leftTrigger().whileTrue(
      Commands.runOnce(() -> {
        manualHopperPopHigh[0] = true;
        m_collector.setHopperPosition(0.19);
        manualShootHopperTimer.reset();
        manualShootHopperTimer.start();
      })
      .andThen(Commands.run(() -> {
        m_shooterSubsystem.runIndexer(false);
        m_shooterSubsystem.runFloor(false);
        m_collector.runCollector(false);
        if (manualShootHopperTimer.advanceIfElapsed(0.25)) {
          manualHopperPopHigh[0] = !manualHopperPopHigh[0];
          m_collector.setHopperPosition(manualHopperPopHigh[0] ? 0.19 : 0.02);
        }
      }, m_shooterSubsystem, m_collector))
      .finallyDo(() -> {
        m_shooterSubsystem.StopIndexer();
        m_shooterSubsystem.StopFloor();
        m_collector.stopCollector();
        manualShootHopperTimer.stop();
      })
    );

    m_mechanismController.rightTrigger().whileTrue(
      new ShooterWithAutoAimCommand(
        m_shooterSubsystem,
        m_swerveDrive,
        m_collector,
        () -> {
          boolean isRed = DriverStation.getAlliance().isPresent() &&
                          DriverStation.getAlliance().get() == Alliance.Red;
          double allianceFlip = isRed ? -1.0 : 1.0;
          return -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)
                 * m_swerveDrive.getMaxSpeedMetersPerSecond() * getSpeedMultiplier() * allianceFlip;
        },
        () -> {
          boolean isRed = DriverStation.getAlliance().isPresent() &&
                          DriverStation.getAlliance().get() == Alliance.Red;
          double allianceFlip = isRed ? -1.0 : 1.0;
          return -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)
                 * m_swerveDrive.getMaxSpeedMetersPerSecond() * getSpeedMultiplier() * allianceFlip;
        }
      )
    );

    // TRAJECTORY TESTING:
    // Left trigger (manual shoot) automatically uses Test RPM when "Test Mode Enabled" is true
    // To test: Enable in Shuffleboard (Shooter/TrajectoryTest/Test Mode Enabled = true)
    //          Set Test RPM and Trajectory Angle, hold left trigger to shoot

  /*
    m_mechanismController.povUp().whileTrue(
      Commands.run(() -> m_climber.setSpeed(0.2), m_climber)
        .finallyDo(() -> m_climber.stop())
    );

    m_mechanismController.povDown().whileTrue(
      Commands.run(() -> m_climber.setSpeed(-0.2), m_climber)
        .finallyDo(() -> m_climber.stop())
    );
  */
  }

  /**
   * Configure shooter test system with NetworkTables entries for test parameters
   */
  private void configureShooterTestSystem() {
    var testTable = NetworkTableInstance.getDefault().getTable("ShooterTest");

    // Create NetworkTables entries for test configuration
    m_testBallsLoadedEntry = testTable.getIntegerTopic("Balls Loaded").getEntry(30);
    m_testBallsScoredEntry = testTable.getIntegerTopic("Balls Scored").getEntry(0);
    m_testDurationEntry = testTable.getDoubleTopic("Test Duration").getEntry(5.0);
    m_testShooterRPMEntry = testTable.getDoubleTopic("Shooter RPM").getEntry(4500);
    m_testIndexerRPMEntry = testTable.getDoubleTopic("Indexer RPM").getEntry(1500);
    m_testMeasuredDistanceEntry = testTable.getDoubleTopic("Measured Distance").getEntry(0.0);

    // Set default values
    m_testBallsLoadedEntry.setDefault(30);
    m_testBallsScoredEntry.setDefault(0);
    m_testDurationEntry.setDefault(5.0);
    m_testShooterRPMEntry.setDefault(4500);
    m_testIndexerRPMEntry.setDefault(1500);
    m_testMeasuredDistanceEntry.setDefault(0.0);

    // NOTE: Left and right bumper bindings for test mode ball scoring have been removed
    // Left bumper is now bound to HopperPopCommand in configureMechanismBindings()

    // Right bumper: Decrement scored count (test mode only)
    m_mechanismController.rightBumper().onTrue(
      Commands.runOnce(() -> {
        int current = (int) m_testBallsScoredEntry.get();
        if (current > 0) {
          m_testBallsScoredEntry.set(current - 1);
        }
      })
    );

    // Back button: Confirm and log test data
    m_mechanismController.back().onTrue(
      Commands.runOnce(() -> {
        if (m_currentTestCommand != null) {
          int ballsScored = (int) m_testBallsScoredEntry.get();
          m_currentTestCommand.logAndFinish(ballsScored);

          // Reset scored count for next test
          m_testBallsScoredEntry.set(0);
        }
      })
    );
  }

  /**
   * Configure alignment PID tuning parameters via NetworkTables
   * Allows real-time tuning without redeploying code
   */
  private void configureAlignmentPIDTuning() {
    var tuningTable = NetworkTableInstance.getDefault().getTable("AlignmentPID");

    // Create NetworkTables entries with default values from AutoConstants
    m_alignmentPEntry = tuningTable.getDoubleTopic("P").getEntry(AutoConstants.kRotateToTargetP);
    m_alignmentIEntry = tuningTable.getDoubleTopic("I").getEntry(AutoConstants.kRotateToTargetI);
    m_alignmentDEntry = tuningTable.getDoubleTopic("D").getEntry(AutoConstants.kRotateToTargetD);
    m_alignmentToleranceEntry = tuningTable.getDoubleTopic("Tolerance (deg)").getEntry(AutoConstants.kRotateToTargetTolerance);

    // Set defaults
    m_alignmentPEntry.setDefault(AutoConstants.kRotateToTargetP);
    m_alignmentIEntry.setDefault(AutoConstants.kRotateToTargetI);
    m_alignmentDEntry.setDefault(AutoConstants.kRotateToTargetD);
    m_alignmentToleranceEntry.setDefault(AutoConstants.kRotateToTargetTolerance);
  }

  /**
   * NOTE: Motor PID tuning has been moved to ShooterDashboard for better organization.
   * This provides all shooter-related configuration in one organized Shuffleboard tab.
   * Access PID tuning via the "Shooter" tab in Shuffleboard under "PID Tuning" layout.
   *
   * BEST PRACTICE: Centralize dashboard configuration in dedicated dashboard classes
   * rather than scattering NetworkTables setup across multiple files. This makes it
   * easier to find values during competition and maintain the codebase.
   */

  /*
   * Configures autonomous commands using Choreo trajectories.
   * Place your .traj files in src/main/deploy/choreo/
   *
   * The chooser will appear on the SmartDashboard/Shuffleboard.
   * Add more paths by creating .traj files and adding them here.
   *
   * To add markers to your trajectory:
   * 1. Open the trajectory in Choreo
   * 2. Add event markers at specific timestamps (e.g., "startCollector", "shoot", "stopCollector")
   * 3. Bind commands to those markers using trajectory.atTime("markerName")
   */
  private void configureAutos() {
    m_autoChooser = new SendableChooser<>();
    m_autoChooser.setDefaultOption("Auto (Pose-Based)", null);
    m_autoChooser.addOption("Red Right Over Hump", new RedRightOverHumpCommand(m_swerveDrive, m_shooterSubsystem, m_collector, m_visionSubsystem));
    m_autoChooser.addOption("Waypoint PID Test", new WaypointPIDTestCommand(m_swerveDrive, m_shooterSubsystem));
    SmartDashboard.putData("Auto/Chooser", m_autoChooser);
    SmartDashboard.putString("Auto/Selected", "Not yet determined");
  }

  /**
   * Updates SmartDashboard with the autonomous routine that would be selected
   * based on current robot position. Call this during disabled periodic for
   * continuous updates.
   */
  public void updateAutoDisplay() {
    if (m_visionSubsystem.getActiveCameraCount() == 0) {
      SmartDashboard.putString("Auto/Selected", "No Vision — Drive and Shoot (No Camera)");
      SmartDashboard.putString("Auto/Alliance", "Unknown");
      SmartDashboard.putString("Auto/Position", "N/A");
      SmartDashboard.putString("Auto/Pose", "No Vision");
      return;
    }

    edu.wpi.first.math.geometry.Pose2d startPose = m_swerveDrive.getPose();
    double x = startPose.getX();
    double y = startPose.getY();
    boolean isRed = x > 8.27;
    String alliance = isRed ? "Red" : "Blue";
    String position;
    String routineName;

    if (isRed) {
      if (y < 3.0) {
        position = "Left";
        routineName = "Red Left Collect and Shoot";
      } else if (y > 5.0) {
        position = "Right";
        routineName = "Red Right Collect and Shoot";
      } else {
        position = "Center";
        routineName = "Red Center Shoot";
      }
    } else {
      if (y > 5.0) {
        position = "Left";
        routineName = "Blue Left Collect and Shoot";
      } else if (y < 3.0) {
        position = "Right";
        routineName = "Blue Right Collect and Shoot";
      } else {
        position = "Center";
        routineName = "Blue Center Shoot";
      }
    }

    SmartDashboard.putString("Auto/Selected", routineName);
    SmartDashboard.putString("Auto/Alliance", alliance);
    SmartDashboard.putString("Auto/Position", position);
    SmartDashboard.putString("Auto/Pose", String.format("(%.2f, %.2f)", x, y));
  }

  /**
   * Selects the autonomous command based on robot starting pose and vision availability.
   * Alliance: x > 8.27 = Red, x <= 8.27 = Blue.
   * Position: y < 3.0 = Left(Red)/Right(Blue), y > 5.0 = Right(Red)/Left(Blue), else Center.
   * If vision is unavailable, drives forward 1 meter and shoots.
   */
  public Command getAutonomousCommand() {
    Command selected = m_autoChooser.getSelected();
    if (selected != null && selected.getName() != null && !selected.getName().equals("none")) {
      return selected;
    }

    if (m_visionSubsystem.getActiveCameraCount() == 0) {
      SmartDashboard.putString("Auto/Selected", "No Vision — Drive and Shoot (No Camera)");
      return new DriveAndShootNoCameraCommand(m_swerveDrive, m_shooterSubsystem, m_collector);
    }

    edu.wpi.first.math.geometry.Pose2d startPose = m_swerveDrive.getPose();
    double x = startPose.getX();
    double y = startPose.getY();
    boolean isRed = x > 8.27;
    String alliance = isRed ? "Red" : "Blue";

    String name;
    Command command;

    if (isRed) {
      if (y < 3.0) {
        name = "Red Left Collect and Shoot";
        command = new RedLeftCollectAndShootCommand(m_swerveDrive, m_shooterSubsystem, m_collector);
      } else if (y > 5.0) {
        name = "Red Right Collect and Shoot";
        command = new RedRightCollectAndShootCommand(m_swerveDrive, m_shooterSubsystem, m_collector);
      } else {
        name = "Red Center Shoot";
        command = new RedCenterShootCommand(m_swerveDrive, m_shooterSubsystem, m_collector);
      }
    } else {
      if (y > 5.0) {
        name = "Blue Left Collect and Shoot";
        command = new BlueLeftCollectAndShootCommand(m_swerveDrive, m_shooterSubsystem, m_collector);
      } else if (y < 3.0) {
        name = "Blue Right Collect and Shoot";
        command = new BlueRightCollectAndShootCommand(m_swerveDrive, m_shooterSubsystem, m_collector);
      } else {
        name = "Blue Center Shoot";
        command = new BlueCenterShootCommand(m_swerveDrive, m_shooterSubsystem, m_collector);
      }
    }

    SmartDashboard.putString("Auto/Selected",
        String.format("%s | Alliance: %s | Pose: (%.2f, %.2f)", name, alliance, x, y));
    return command;
  }

  /**
   * Spin up shooter on teleop enable (before driving starts)
   * This uses the high startup current before the drivetrain demands power
   */
  public void spinUpShooter() {
    m_shooterSubsystem.activateShooter();
  }

  public DriveSubsystem getSwerveDrive() {
    return m_swerveDrive;
  }

  public PhotonVisionSubsystem getVisionSubsystem() {
    return m_visionSubsystem;
  }

  public LEDSubsystem getLEDSubsystem() {
    return m_ledSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  public CollectorSubsystem getCollector() {
    return m_collector;
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

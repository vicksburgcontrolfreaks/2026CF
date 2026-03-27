// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.collector.RunCollectorCommand;
import frc.robot.commands.collector.StopCollectorCommand;
import frc.robot.commands.collector.ExtendHopperCommand;
import frc.robot.commands.collector.RetractHopperCommand;
import frc.robot.commands.drive.RotateToTargetCommand;
import frc.robot.commands.led.AprilTagLEDCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.test.ShooterTestCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.utils.ShooterTestLogger;
import choreo.auto.AutoFactory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.DoubleEntry;


public class RobotContainer {
  private final DriveSubsystem m_swerveDrive = new DriveSubsystem();
  private final PhotonVisionSubsystem m_visionSubsystem = new PhotonVisionSubsystem(m_swerveDrive);
  private final LEDSubsystem m_ledSubsystem = null;
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(m_visionSubsystem);
  private final CollectorSubsystem m_collector = new CollectorSubsystem();
  private final ClimberSubsystem m_climber = null;

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_mechanismController;

  private final AutoFactory autoFactory;
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

  // Shooter motor PID tuning (public for ShooterSubsystem access)
  public DoubleEntry m_shooterPEntry;
  public DoubleEntry m_shooterIEntry;
  public DoubleEntry m_shooterDEntry;
  public DoubleEntry m_shooterFFEntry;

  // Indexer motor PID tuning (public for ShooterSubsystem access)
  public DoubleEntry m_indexerPEntry;
  public DoubleEntry m_indexerIEntry;
  public DoubleEntry m_indexerDEntry;
  public DoubleEntry m_indexerFFEntry;

  public RobotContainer() {
      m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
      m_mechanismController = new CommandXboxController(OIConstants.kMechanismControllerPort);

      autoFactory = new AutoFactory(
          m_swerveDrive::getPose,
          m_swerveDrive::resetOdometry,
          m_swerveDrive::followTrajectory,
          false,
          m_swerveDrive
      );

      configureShooterTestSystem();
      configureAlignmentPIDTuning();
      configureMotorPIDTuning();
      configureAutos();

      // Connect shooter subsystem to this container for PID tuning access
      m_shooterSubsystem.setContainer(this);

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

          // Read raw joystick inputs
          double xSpeed = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)
                          * DriveConstants.kMaxSpeedMetersPerSecond * speedMultiplier;
          double ySpeed = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)
                          * DriveConstants.kMaxSpeedMetersPerSecond * speedMultiplier;
          double rotSpeed = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)
                            * DriveConstants.kMaxAngularSpeed * speedMultiplier;

          // Alliance-relative driving: flip X and Y when on red alliance
          // This makes "forward" always mean "toward opponent"
          if (DriverStation.getAlliance().isPresent() &&
              DriverStation.getAlliance().get() == Alliance.Red) {
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
          }

          m_swerveDrive.drive(xSpeed, ySpeed, rotSpeed, true);
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
    // A Button: Run collector in collection mode (forward)
    m_mechanismController.a().whileTrue(
      new RunCollectorCommand(m_collector, false).alongWith(Commands.run(() -> {
        m_shooterSubsystem.runFloor(false);
        m_shooterSubsystem.runIndexer(true);
      }, m_shooterSubsystem))
    );

    // X Button: Reverse collector (unjam/eject)
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

    m_mechanismController.povUp().onTrue(
      new RetractHopperCommand(m_collector)
    );

    m_mechanismController.povDown().onTrue(
      new ExtendHopperCommand(m_collector)
    );

    m_mechanismController.rightTrigger().whileTrue(
      new ShooterCommand(m_shooterSubsystem, m_visionSubsystem, m_swerveDrive, m_driverController, this)
    );

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

    // Add controller bindings for scored ball input during test mode
    // Left bumper: Increment scored count
    m_mechanismController.leftBumper().onTrue(
      Commands.runOnce(() -> {
        int current = (int) m_testBallsScoredEntry.get();
        m_testBallsScoredEntry.set(current + 1);
      })
    );

    // Right bumper: Decrement scored count
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
   * Configure shooter and indexer motor PID tuning parameters via NetworkTables
   * Allows real-time tuning of motor velocity control without redeploying code
   */
  private void configureMotorPIDTuning() {
    var shooterTable = NetworkTableInstance.getDefault().getTable("ShooterMotorPID");
    var indexerTable = NetworkTableInstance.getDefault().getTable("IndexerMotorPID");

    // Shooter motor PID
    m_shooterPEntry = shooterTable.getDoubleTopic("P").getEntry(ShooterConstants.kShooterP);
    m_shooterIEntry = shooterTable.getDoubleTopic("I").getEntry(ShooterConstants.kShooterI);
    m_shooterDEntry = shooterTable.getDoubleTopic("D").getEntry(ShooterConstants.kShooterD);
    m_shooterFFEntry = shooterTable.getDoubleTopic("FF").getEntry(ShooterConstants.kShooterFF);

    m_shooterPEntry.setDefault(ShooterConstants.kShooterP);
    m_shooterIEntry.setDefault(ShooterConstants.kShooterI);
    m_shooterDEntry.setDefault(ShooterConstants.kShooterD);
    m_shooterFFEntry.setDefault(ShooterConstants.kShooterFF);

    // Indexer motor PID
    m_indexerPEntry = indexerTable.getDoubleTopic("P").getEntry(ShooterConstants.kIndexerP);
    m_indexerIEntry = indexerTable.getDoubleTopic("I").getEntry(ShooterConstants.kIndexerI);
    m_indexerDEntry = indexerTable.getDoubleTopic("D").getEntry(ShooterConstants.kIndexerD);
    m_indexerFFEntry = indexerTable.getDoubleTopic("FF").getEntry(ShooterConstants.kIndexerFF);

    m_indexerPEntry.setDefault(ShooterConstants.kIndexerP);
    m_indexerIEntry.setDefault(ShooterConstants.kIndexerI);
    m_indexerDEntry.setDefault(ShooterConstants.kIndexerD);
    m_indexerFFEntry.setDefault(ShooterConstants.kIndexerFF);
  }

  /*
   * Configures autonomous commands using Choreo trajectories.
   * Place your .traj files in src/main/deploy/choreo/
   *
   * The chooser will appear on the SmartDashboard/Shuffleboard.
   * Add more paths by creating .traj files and adding them here.
   */
  private void configureAutos() {
    m_autoChooser = new SendableChooser<>();

    try {
      // Add autonomous options to the chooser
      // Format: m_autoChooser.addOption("Display Name", autoFactory.trajectoryCmd("PathName"));

      m_autoChooser.setDefaultOption("Red Right Loop", autoFactory.trajectoryCmd("RedRightLoop"));
      m_autoChooser.addOption("Do Nothing", Commands.none());

      // Add shooter test command
      m_autoChooser.addOption("Shooter Test", Commands.defer(() -> {
        int ballsLoaded = (int) m_testBallsLoadedEntry.get();
        double duration = m_testDurationEntry.get();
        double shooterRPM = m_testShooterRPMEntry.get();
        double indexerRPM = m_testIndexerRPMEntry.get();
        double measuredDistance = m_testMeasuredDistanceEntry.get();
        m_currentTestCommand = new ShooterTestCommand(m_shooterSubsystem, m_shooterTestLogger,
            ballsLoaded, duration, shooterRPM, indexerRPM, measuredDistance);
        return m_currentTestCommand;
      }, Set.of(m_shooterSubsystem)));

    } catch (Exception e) {
      System.err.println("Failed to load Choreo trajectory: " + e.getMessage());
      System.err.println("Make sure you have .traj files in src/main/deploy/choreo/");
      e.printStackTrace();
      m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
    }

    // Send the chooser to SmartDashboard (which uses NetworkTables under the hood)
    SmartDashboard.putData("Auto/Chooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
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
}

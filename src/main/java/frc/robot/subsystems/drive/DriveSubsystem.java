// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TelemetryConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import java.io.IOException;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Field2d for visualizing robot pose in Glass
  private final Field2d m_field = new Field2d();

  private int m_telemetryCounter = 0;

  // NetworkTable for gyro logging
  private final NetworkTable m_gyroTable = NetworkTableInstance.getDefault().getTable("Gyro");
  private final DoublePublisher m_xAnglePub;
  private final DoublePublisher m_yAnglePub;
  private final DoublePublisher m_zAnglePub;
  private final DoublePublisher m_xRatePub;
  private final DoublePublisher m_yRatePub;
  private final DoublePublisher m_zRatePub;
  private final DoublePublisher m_headingPub;
  private final DoublePublisher m_currentXPub;
  private final DoublePublisher m_currentYPub;
  // Pose estimator for tracking robot pose with vision fusion
  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kY) * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d());

  // Configurable constants (via NetworkTables)
  private double m_maxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond;
  private double m_maxAngularSpeed = DriveConstants.kMaxAngularSpeed;
  private int m_telemetryUpdatePeriod = TelemetryConstants.kTelemetryUpdatePeriod;

  // NetworkTables subscribers for configurable constants
  private final DoubleSubscriber m_maxSpeedMetersPerSecondSub;
  private final DoubleSubscriber m_maxAngularSpeedSub;
  private final DoubleSubscriber m_telemetryUpdatePeriodSub;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // Initialize telemetry publishers
    m_xAnglePub = m_gyroTable.getDoubleTopic("X Angle").publish();
    m_yAnglePub = m_gyroTable.getDoubleTopic("Y Angle").publish();
    m_zAnglePub = m_gyroTable.getDoubleTopic("Z Angle").publish();
    m_xRatePub = m_gyroTable.getDoubleTopic("X Rate").publish();
    m_yRatePub = m_gyroTable.getDoubleTopic("Y Rate").publish();
    m_zRatePub = m_gyroTable.getDoubleTopic("Z Rate").publish();
    m_headingPub = m_gyroTable.getDoubleTopic("Heading").publish();
    m_currentXPub = m_gyroTable.getDoubleTopic("Current X").publish();
    m_currentYPub = m_gyroTable.getDoubleTopic("Current Y").publish();

    // Publish Field2d to SmartDashboard for Glass visualization
    SmartDashboard.putData("Field", m_field);

    // Initialize NetworkTables subscribers for configurable constants
    NetworkTable configTable = NetworkTableInstance.getDefault().getTable("Drive/Config");
    m_maxSpeedMetersPerSecondSub = configTable.getDoubleTopic("Max Speed Meters Per Second").subscribe(m_maxSpeedMetersPerSecond);
    m_maxAngularSpeedSub = configTable.getDoubleTopic("Max Angular Speed").subscribe(m_maxAngularSpeed);
    m_telemetryUpdatePeriodSub = configTable.getDoubleTopic("Telemetry Update Period").subscribe(m_telemetryUpdatePeriod);

    // Configure PathPlanner AutoBuilder
    configurePathPlanner();
  }

  /**
   * Configures PathPlanner's AutoBuilder for autonomous path following.
   * This must be called after the drivetrain is initialized.
   */
  private void configurePathPlanner() {
    try {
      // Load the robot configuration from deploy directory
      // This file is generated by PathPlanner GUI
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder with holonomic drive controller
      AutoBuilder.configure(
          this::getPose, // Supplier of current robot pose
          this::resetOdometry, // Consumer to reset odometry
          this::getChassisSpeeds, // Supplier of current robot-relative chassis speeds
          (speeds, feedforwards) -> driveRobotRelative(speeds), // Consumer of ChassisSpeeds to drive the robot
          new PPHolonomicDriveController(
              // PID constants for path following (tune these for your robot)
              DriveConstants.kPathFollowingTranslationPID,
              DriveConstants.kPathFollowingRotationPID
          ),
          config, // Robot configuration
          () -> {
            // Boolean supplier that returns true if path should be flipped for red alliance
            // PathPlanner paths are designed for blue alliance by default
            var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
          },
          this // Subsystem requirement
      );
    } catch (IOException e) {
      System.err.println("Failed to load PathPlanner config: " + e.getMessage());
      e.printStackTrace();
    } catch (Exception e) {
      System.err.println("Failed to configure PathPlanner: " + e.getMessage());
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // Read configurable values from NetworkTables
    m_maxSpeedMetersPerSecond = m_maxSpeedMetersPerSecondSub.get();
    m_maxAngularSpeed = m_maxAngularSpeedSub.get();
    m_telemetryUpdatePeriod = (int) m_telemetryUpdatePeriodSub.get();

    // Update the pose estimator in the periodic block
    m_poseEstimator.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kY) * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Update Field2d with current robot pose for Glass visualization
    m_field.setRobotPose(getPose());

    // Throttled telemetry updates
    m_telemetryCounter++;
    if (m_telemetryCounter >= getTelemetryUpdatePeriod()) {
      m_telemetryCounter = 0;

      m_xAnglePub.set(m_gyro.getAngle(IMUAxis.kX));
      m_yAnglePub.set(m_gyro.getAngle(IMUAxis.kY));
      m_zAnglePub.set(m_gyro.getAngle(IMUAxis.kZ));
      m_xRatePub.set(m_gyro.getRate(IMUAxis.kX));
      m_yRatePub.set(m_gyro.getRate(IMUAxis.kY));
      m_zRatePub.set(m_gyro.getRate(IMUAxis.kZ));
      m_headingPub.set(getHeading());
      m_currentXPub.set(getPose().getX());
      m_currentYPub.set(getPose().getY());
    }
  }

  // Getters and setters for configurable constants
  public double getMaxSpeedMetersPerSecond() {
    return m_maxSpeedMetersPerSecond;
  }

  public void setMaxSpeedMetersPerSecond(double speed) {
    m_maxSpeedMetersPerSecond = speed;
  }

  public double getMaxAngularSpeed() {
    return m_maxAngularSpeed;
  }

  public void setMaxAngularSpeed(double speed) {
    m_maxAngularSpeed = speed;
  }

  public int getTelemetryUpdatePeriod() {
    return m_telemetryUpdatePeriod;
  }

  public void setTelemetryUpdatePeriod(int period) {
    m_telemetryUpdatePeriod = period;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   * Also resets the gyro to match the pose rotation, eliminating any offset.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // Reset gyro to match the desired pose rotation
    // This eliminates offset between gyro and vision measurements
    m_gyro.setGyroAngleY(pose.getRotation().getDegrees() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));

    m_poseEstimator.resetPosition(
        pose.getRotation(),  // Use the desired rotation directly
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by vision
   * @param timestampSeconds The timestamp of the vision measurement in seconds
   * @param visionMeasurementStdDevs Standard deviations of the vision measurement
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    m_poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters,
        timestampSeconds,
        visionMeasurementStdDevs);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward) in m/s.
   * @param ySpeed        Speed of the robot in the y direction (sideways) in m/s.
   * @param rot           Angular rate of the robot in rad/s.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Create chassis speeds from inputs
    // Use pose estimator rotation (fused with vision) instead of raw gyro for field-relative
    ChassisSpeeds speeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
            getPose().getRotation())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    // Discretize chassis speeds to prevent skew/drift during motion
    // 0.02 seconds = 20ms loop time (50Hz)
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot by resetting both gyro and pose estimator.
   * This ensures field-oriented drive treats the current direction as "forward".
   */
  public void zeroHeading() {
    m_gyro.reset();
    // Also reset the pose estimator to match, keeping current X/Y position
    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(0.0),  // Gyro now reads 0
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(0.0))  // Keep position, reset rotation to 0
    );
  }

  /**
   * Returns the heading of the robot from the pose estimator (fused with vision).
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getPose().getRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kY) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Stops all drive motors.
   */
  public void stop() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
    m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
    m_rearRight.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
  }

  /**
   * Returns the current robot-relative chassis speeds.
   * Used by PathPlanner for autonomous path following.
   *
   * @return The current chassis speeds.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    );
  }

  /**
   * Drives the robot using robot-relative chassis speeds.
   * Used by PathPlanner for autonomous path following.
   *
   * @param speeds The desired robot-relative chassis speeds.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Discretize chassis speeds to prevent skew/drift during motion
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  public MAXSwerveModule getFrontLeftDriveMotor() {
    return m_frontLeft;
  }

  public MAXSwerveModule getFrontRightDriveMotor() {
    return m_frontRight;
  }

  public MAXSwerveModule getRearLeftDriveMotor() {
    return m_rearLeft;
  }

  public MAXSwerveModule getRearRightDriveMotor() {
    return m_rearRight;
  }

}

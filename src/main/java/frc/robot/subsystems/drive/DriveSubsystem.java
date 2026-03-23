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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TelemetryConstants;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;

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
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kY)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d());

  // PID controllers for Choreo trajectory following
  private final PIDController m_xController = new PIDController(5.0, 0, 0);
  private final PIDController m_yController = new PIDController(5.0, 0, 0);
  private final PIDController m_thetaController = new PIDController(3.0, 0, 0);

  // PID controller for heading lock to prevent drift
  private final PIDController m_headingController = new PIDController(0.04, 0, 0);
  private double m_targetHeading = 0;
  private boolean m_wasRotating = false;

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
    // Configure theta controller for continuous input
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Configure heading controller for continuous input (handles 0-360 wraparound)
    m_headingController.enableContinuousInput(-180, 180);

    // Publish Field2d to SmartDashboard for Glass visualization
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // Update the pose estimator in the periodic block
    m_poseEstimator.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kY)),
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
    if (m_telemetryCounter >= TelemetryConstants.kTelemetryUpdatePeriod) {
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
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kY)),
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
    // Apply heading lock only when there's no rotation input AND robot is moving
    // This prevents drift while driving straight
    if (Math.abs(rot) < 0.05) { // If rotation input is within deadband
      // If we just stopped rotating, update target to current heading before locking
      if (m_wasRotating) {
        m_targetHeading = m_gyro.getAngle(IMUAxis.kY);
        m_wasRotating = false;
      }

      // Only apply heading correction if the robot is actually moving
      if (Math.abs(xSpeed) > 0.1 || Math.abs(ySpeed) > 0.1) {
        double currentHeading = m_gyro.getAngle(IMUAxis.kY);
        rot = m_headingController.calculate(currentHeading, m_targetHeading);
      }
    } else {
      // Driver is actively rotating
      m_wasRotating = true;
      m_targetHeading = m_gyro.getAngle(IMUAxis.kY);
    }

    // Create chassis speeds from inputs
    ChassisSpeeds speeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
            Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kY)))
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

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_targetHeading = 0;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kY)).getDegrees();
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
   * Sets the chassis speeds for autonomous path following.
   * Used by Choreo for trajectory following.
   *
   * @param speeds The desired chassis speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Returns the current robot ChassisSpeeds.
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
   * Follows a Choreo trajectory sample.
   * This method is called by the AutoFactory to control the robot.
   *
   * @param sample The current trajectory sample to follow.
   */
  public void followTrajectory(SwerveSample sample) {
    // Get current robot pose
    Pose2d currentPose = getPose();

    // Calculate PID feedback (in field coordinates)
    double xFeedback = m_xController.calculate(currentPose.getX(), sample.x);
    double yFeedback = m_yController.calculate(currentPose.getY(), sample.y);
    double thetaFeedback = m_thetaController.calculate(
        currentPose.getRotation().getRadians(),
        sample.heading
    );

    // Convert field-relative PID feedback to robot-relative
    ChassisSpeeds feedbackSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xFeedback, yFeedback, thetaFeedback,
        currentPose.getRotation()
    );

    // Combine robot-relative feedforward from Choreo with robot-relative PID feedback
    // Note: Choreo provides robot-relative velocities (vx, vy, omega)
    ChassisSpeeds targetSpeeds = new ChassisSpeeds(
        sample.vx + feedbackSpeeds.vxMetersPerSecond,
        sample.vy + feedbackSpeeds.vyMetersPerSecond,
        sample.omega + feedbackSpeeds.omegaRadiansPerSecond
    );

    // Send robot-relative speeds to the drivetrain
    setChassisSpeeds(targetSpeeds);
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

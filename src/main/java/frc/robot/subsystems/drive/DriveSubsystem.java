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
  private final PIDController m_xController = new PIDController(8.0, 0, 0.3);
  private final PIDController m_yController = new PIDController(8.0, 0, 0.3);
  private final PIDController m_thetaController = new PIDController(5.0, 0, 0);

  // Configurable constants (via NetworkTables)
  private double m_maxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond;
  private double m_maxAngularSpeed = DriveConstants.kMaxAngularSpeed;
  private double m_xControllerP = 5.0;
  private double m_xControllerI = 0.0;
  private double m_xControllerD = 0.0;
  private double m_yControllerP = 5.0;
  private double m_yControllerI = 0.0;
  private double m_yControllerD = 0.0;
  private double m_thetaControllerP = 3.0;
  private double m_thetaControllerI = 0.0;
  private double m_thetaControllerD = 0.0;
  private int m_telemetryUpdatePeriod = TelemetryConstants.kTelemetryUpdatePeriod;

  // NetworkTables subscribers for configurable constants
  private final DoubleSubscriber m_maxSpeedMetersPerSecondSub;
  private final DoubleSubscriber m_maxAngularSpeedSub;
  private final DoubleSubscriber m_xControllerPSub;
  private final DoubleSubscriber m_xControllerISub;
  private final DoubleSubscriber m_xControllerDSub;
  private final DoubleSubscriber m_yControllerPSub;
  private final DoubleSubscriber m_yControllerISub;
  private final DoubleSubscriber m_yControllerDSub;
  private final DoubleSubscriber m_thetaControllerPSub;
  private final DoubleSubscriber m_thetaControllerISub;
  private final DoubleSubscriber m_thetaControllerDSub;
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
    // Configure theta controller for continuous input
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Publish Field2d to SmartDashboard for Glass visualization
    SmartDashboard.putData("Field", m_field);

    // Initialize NetworkTables subscribers for configurable constants
    NetworkTable configTable = NetworkTableInstance.getDefault().getTable("Drive/Config");
    m_maxSpeedMetersPerSecondSub = configTable.getDoubleTopic("Max Speed Meters Per Second").subscribe(m_maxSpeedMetersPerSecond);
    m_maxAngularSpeedSub = configTable.getDoubleTopic("Max Angular Speed").subscribe(m_maxAngularSpeed);
    m_xControllerPSub = configTable.getDoubleTopic("X Controller P").subscribe(m_xControllerP);
    m_xControllerISub = configTable.getDoubleTopic("X Controller I").subscribe(m_xControllerI);
    m_xControllerDSub = configTable.getDoubleTopic("X Controller D").subscribe(m_xControllerD);
    m_yControllerPSub = configTable.getDoubleTopic("Y Controller P").subscribe(m_yControllerP);
    m_yControllerISub = configTable.getDoubleTopic("Y Controller I").subscribe(m_yControllerI);
    m_yControllerDSub = configTable.getDoubleTopic("Y Controller D").subscribe(m_yControllerD);
    m_thetaControllerPSub = configTable.getDoubleTopic("Theta Controller P").subscribe(m_thetaControllerP);
    m_thetaControllerISub = configTable.getDoubleTopic("Theta Controller I").subscribe(m_thetaControllerI);
    m_thetaControllerDSub = configTable.getDoubleTopic("Theta Controller D").subscribe(m_thetaControllerD);
    m_telemetryUpdatePeriodSub = configTable.getDoubleTopic("Telemetry Update Period").subscribe(m_telemetryUpdatePeriod);
  }

  @Override
  public void periodic() {
    // Read configurable values from NetworkTables
    m_maxSpeedMetersPerSecond = m_maxSpeedMetersPerSecondSub.get();
    m_maxAngularSpeed = m_maxAngularSpeedSub.get();

    double newXP = m_xControllerPSub.get();
    double newXI = m_xControllerISub.get();
    double newXD = m_xControllerDSub.get();
    if (newXP != m_xControllerP || newXI != m_xControllerI || newXD != m_xControllerD) {
      m_xControllerP = newXP;
      m_xControllerI = newXI;
      m_xControllerD = newXD;
      m_xController.setPID(m_xControllerP, m_xControllerI, m_xControllerD);
    }

    double newYP = m_yControllerPSub.get();
    double newYI = m_yControllerISub.get();
    double newYD = m_yControllerDSub.get();
    if (newYP != m_yControllerP || newYI != m_yControllerI || newYD != m_yControllerD) {
      m_yControllerP = newYP;
      m_yControllerI = newYI;
      m_yControllerD = newYD;
      m_yController.setPID(m_yControllerP, m_yControllerI, m_yControllerD);
    }

    double newThetaP = m_thetaControllerPSub.get();
    double newThetaI = m_thetaControllerISub.get();
    double newThetaD = m_thetaControllerDSub.get();
    if (newThetaP != m_thetaControllerP || newThetaI != m_thetaControllerI || newThetaD != m_thetaControllerD) {
      m_thetaControllerP = newThetaP;
      m_thetaControllerI = newThetaI;
      m_thetaControllerD = newThetaD;
      m_thetaController.setPID(m_thetaControllerP, m_thetaControllerI, m_thetaControllerD);
    }

    m_telemetryUpdatePeriod = (int) m_telemetryUpdatePeriodSub.get();

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

  public double getXControllerP() {
    return m_xControllerP;
  }

  public void setXControllerP(double p) {
    m_xControllerP = p;
    m_xController.setP(p);
  }

  public double getXControllerI() {
    return m_xControllerI;
  }

  public void setXControllerI(double i) {
    m_xControllerI = i;
    m_xController.setI(i);
  }

  public double getXControllerD() {
    return m_xControllerD;
  }

  public void setXControllerD(double d) {
    m_xControllerD = d;
    m_xController.setD(d);
  }

  public double getYControllerP() {
    return m_yControllerP;
  }

  public void setYControllerP(double p) {
    m_yControllerP = p;
    m_yController.setP(p);
  }

  public double getYControllerI() {
    return m_yControllerI;
  }

  public void setYControllerI(double i) {
    m_yControllerI = i;
    m_yController.setI(i);
  }

  public double getYControllerD() {
    return m_yControllerD;
  }

  public void setYControllerD(double d) {
    m_yControllerD = d;
    m_yController.setD(d);
  }

  public double getThetaControllerP() {
    return m_thetaControllerP;
  }

  public void setThetaControllerP(double p) {
    m_thetaControllerP = p;
    m_thetaController.setP(p);
  }

  public double getThetaControllerI() {
    return m_thetaControllerI;
  }

  public void setThetaControllerI(double i) {
    m_thetaControllerI = i;
    m_thetaController.setI(i);
  }

  public double getThetaControllerD() {
    return m_thetaControllerD;
  }

  public void setThetaControllerD(double d) {
    m_thetaControllerD = d;
    m_thetaController.setD(d);
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

    // Calculate PID feedback for position tracking (in field coordinates)
    double xFeedback = m_xController.calculate(currentPose.getX(), sample.x);
    double yFeedback = m_yController.calculate(currentPose.getY(), sample.y);
    double thetaFeedback = m_thetaController.calculate(
        currentPose.getRotation().getRadians(),
        sample.heading
    );

    // Choreo provides field-relative velocities in SwerveSample
    // Combine feedforward from trajectory with PID feedback, both in field coordinates
    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        sample.vx + xFeedback,
        sample.vy + yFeedback,
        sample.omega + thetaFeedback,
        currentPose.getRotation()
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

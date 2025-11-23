// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final ADIS16470_IMU m_gyro;

  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final Field2d m_field;

  private boolean m_fieldOriented = true;

  public SwerveDrive() {
    // Initialize swerve modules
    m_frontLeft = new SwerveModule(
      SwerveConstants.kFrontLeftDriveMotorId,
      SwerveConstants.kFrontLeftSteerMotorId,
      SwerveConstants.kFrontLeftOffset,
      "Front Left"
    );

    m_frontRight = new SwerveModule(
      SwerveConstants.kFrontRightDriveMotorId,
      SwerveConstants.kFrontRightSteerMotorId,
      SwerveConstants.kFrontRightOffset,
      "Front Right"
    );

    m_backLeft = new SwerveModule(
      SwerveConstants.kBackLeftDriveMotorId,
      SwerveConstants.kBackLeftSteerMotorId,
      SwerveConstants.kBackLeftOffset,
      "Back Left"
    );

    m_backRight = new SwerveModule(
      SwerveConstants.kBackRightDriveMotorId,
      SwerveConstants.kBackRightSteerMotorId,
      SwerveConstants.kBackRightOffset,
      "Back Right"
    );

    // Initialize ADIS16470 IMU
    // Using default SPI port (onboard SPI) and Z-axis (yaw)
    m_gyro = new ADIS16470_IMU();
    m_gyro.reset();

    // Calibrate the gyro (robot must be stationary)
    m_gyro.calibrate();

    // Initialize pose estimator
    m_poseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.kSwerveKinematics,
      getGyroRotation2d(),
      getModulePositions(),
      new Pose2d()
    );

    // Initialize field visualization
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // Update pose estimator with current module positions and gyro reading
    m_poseEstimator.update(getGyroRotation2d(), getModulePositions());

    // Update field visualization
    m_field.setRobotPose(getPose());

    // Publish telemetry to SmartDashboard
    SmartDashboard.putNumber("Gyro Angle", getHeading());
    SmartDashboard.putString("Robot Pose", getPose().toString());
    SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);

    SmartDashboard.putNumber("FL Velocity", m_frontLeft.getDriveVelocity());
    SmartDashboard.putNumber("FR Velocity", m_frontRight.getDriveVelocity());
    SmartDashboard.putNumber("BL Velocity", m_backLeft.getDriveVelocity());
    SmartDashboard.putNumber("BR Velocity", m_backRight.getDriveVelocity());

    SmartDashboard.putNumber("FL Angle", Math.toDegrees(m_frontLeft.getSteerPosition()));
    SmartDashboard.putNumber("FR Angle", Math.toDegrees(m_frontRight.getSteerPosition()));
    SmartDashboard.putNumber("BL Angle", Math.toDegrees(m_backLeft.getSteerPosition()));
    SmartDashboard.putNumber("BR Angle", Math.toDegrees(m_backRight.getSteerPosition()));

    // Publish raw absolute encoder values for calibration
    SmartDashboard.putNumber("FL Absolute Encoder", m_frontLeft.getAbsoluteEncoderRaw() / (2 * Math.PI));
    SmartDashboard.putNumber("FR Absolute Encoder", m_frontRight.getAbsoluteEncoderRaw() / (2 * Math.PI));
    SmartDashboard.putNumber("BL Absolute Encoder", m_backLeft.getAbsoluteEncoderRaw() / (2 * Math.PI));
    SmartDashboard.putNumber("BR Absolute Encoder", m_backRight.getAbsoluteEncoderRaw() / (2 * Math.PI));
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert chassis speeds to module states
    SwerveModuleState[] swerveModuleStates;

    if (fieldRelative) {
      swerveModuleStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroRotation2d())
      );
    } else {
      swerveModuleStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, rot)
      );
    }

    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Normalize wheel speeds to be at or below max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      SwerveConstants.kMaxSpeedMetersPerSecond
    );

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
      getGyroRotation2d(),
      getModulePositions(),
      pose
    );
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
  }

  public void addVisionMeasurement(
      Pose2d visionPose,
      double timestampSeconds,
      edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> visionMeasurementStdDevs) {
    m_poseEstimator.addVisionMeasurement(
      visionPose,
      timestampSeconds,
      visionMeasurementStdDevs
    );
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public double getHeading() {
    // ADIS16470 getAngle returns counter-clockwise positive, negate for standard convention
    return Math.IEEEremainder(-m_gyro.getAngle(), 360);
  }

  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void setFieldOriented(boolean fieldOriented) {
    m_fieldOriented = fieldOriented;
  }

  public boolean isFieldOriented() {
    return m_fieldOriented;
  }

  public void toggleFieldOriented() {
    m_fieldOriented = !m_fieldOriented;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(SwerveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
}

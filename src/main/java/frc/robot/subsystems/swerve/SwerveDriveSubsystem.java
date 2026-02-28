// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TelemetryConstants;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final ADIS16470_IMU m_gyro;

  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final Field2d m_field;

  // Elastic (NetworkTables) publishers for telemetry
  private final NetworkTable m_telemetryTable;
  private final DoublePublisher m_gyroAnglePub;
  private final StringPublisher m_robotPosePub;
  private final DoublePublisher m_flVelocityPub;
  private final DoublePublisher m_frVelocityPub;
  private final DoublePublisher m_blVelocityPub;
  private final DoublePublisher m_brVelocityPub;
  private final DoublePublisher m_flAnglePub;
  private final DoublePublisher m_frAnglePub;
  private final DoublePublisher m_blAnglePub;
  private final DoublePublisher m_brAnglePub;
  private final DoublePublisher m_flAbsEncoderPub;
  private final DoublePublisher m_frAbsEncoderPub;
  private final DoublePublisher m_blAbsEncoderPub;
  private final DoublePublisher m_brAbsEncoderPub;

  private int m_telemetryCounter = 0;

  public SwerveDriveSubsystem() {
    m_frontLeft = new SwerveModule(
      SwerveConstants.kFrontLeftDriveMotorId,
      SwerveConstants.kFrontLeftSteerMotorId,
      SwerveConstants.kFrontLeftChassisAngularOffset
    );

    m_frontRight = new SwerveModule(
      SwerveConstants.kFrontRightDriveMotorId,
      SwerveConstants.kFrontRightSteerMotorId,
      SwerveConstants.kFrontRightChassisAngularOffset
    );

    m_backLeft = new SwerveModule(
      SwerveConstants.kBackLeftDriveMotorId,
      SwerveConstants.kBackLeftSteerMotorId,
      SwerveConstants.kBackLeftChassisAngularOffset
    );

    m_backRight = new SwerveModule(
      SwerveConstants.kBackRightDriveMotorId,
      SwerveConstants.kBackRightSteerMotorId,
      SwerveConstants.kBackRightChassisAngularOffset
    );

    System.out.println("==============================================");
    System.out.println("GYRO CALIBRATION STARTING - DO NOT MOVE ROBOT");
    System.out.println("Calibration time: " + SwerveConstants.kGyroCalibrationTimeSec + " seconds");
    System.out.println("==============================================");

    m_gyro = new ADIS16470_IMU(
      IMUAxis.kY,
      IMUAxis.kZ,
      IMUAxis.kX,
      SPI.Port.kOnboardCS0,
      CalibrationTime._2s
    );

    System.out.println("==============================================");
    System.out.println("GYRO CALIBRATION COMPLETE");
    System.out.println("==============================================");

    m_gyro.reset();

    m_poseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.kSwerveKinematics,
      getGyroRotation2d(),
      getModulePositions(),
      new Pose2d()
    );

    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    m_telemetryTable = NetworkTableInstance.getDefault().getTable("Swerve");
    m_gyroAnglePub = m_telemetryTable.getDoubleTopic("Gyro Angle").publish();
    m_robotPosePub = m_telemetryTable.getStringTopic("Robot Pose").publish();
    m_flVelocityPub = m_telemetryTable.getDoubleTopic("FL Velocity").publish();
    m_frVelocityPub = m_telemetryTable.getDoubleTopic("FR Velocity").publish();
    m_blVelocityPub = m_telemetryTable.getDoubleTopic("BL Velocity").publish();
    m_brVelocityPub = m_telemetryTable.getDoubleTopic("BR Velocity").publish();
    m_flAnglePub = m_telemetryTable.getDoubleTopic("FL Angle").publish();
    m_frAnglePub = m_telemetryTable.getDoubleTopic("FR Angle").publish();
    m_blAnglePub = m_telemetryTable.getDoubleTopic("BL Angle").publish();
    m_brAnglePub = m_telemetryTable.getDoubleTopic("BR Angle").publish();
    m_flAbsEncoderPub = m_telemetryTable.getDoubleTopic("FL Absolute Encoder").publish();
    m_frAbsEncoderPub = m_telemetryTable.getDoubleTopic("FR Absolute Encoder").publish();
    m_blAbsEncoderPub = m_telemetryTable.getDoubleTopic("BL Absolute Encoder").publish();
    m_brAbsEncoderPub = m_telemetryTable.getDoubleTopic("BR Absolute Encoder").publish();
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(getGyroRotation2d(), getModulePositions());
    m_field.setRobotPose(getPose());

    m_gyroAnglePub.set(getHeading());
    m_robotPosePub.set(getPose().toString());

    m_telemetryCounter++;
    if (m_telemetryCounter >= TelemetryConstants.kTelemetryUpdatePeriod) {
      m_telemetryCounter = 0;

      m_flVelocityPub.set(m_frontLeft.getDriveVelocity());
      m_frVelocityPub.set(m_frontRight.getDriveVelocity());
      m_blVelocityPub.set(m_backLeft.getDriveVelocity());
      m_brVelocityPub.set(m_backRight.getDriveVelocity());

      m_flAnglePub.set(Math.toDegrees(m_frontLeft.getSteerPosition()));
      m_frAnglePub.set(Math.toDegrees(m_frontRight.getSteerPosition()));
      m_blAnglePub.set(Math.toDegrees(m_backLeft.getSteerPosition()));
      m_brAnglePub.set(Math.toDegrees(m_backRight.getSteerPosition()));

      m_flAbsEncoderPub.set(m_frontLeft.getAbsoluteEncoderRaw() / SwerveConstants.kEncoderNormalization);
      m_frAbsEncoderPub.set(m_frontRight.getAbsoluteEncoderRaw() / SwerveConstants.kEncoderNormalization);
      m_blAbsEncoderPub.set(m_backLeft.getAbsoluteEncoderRaw() / SwerveConstants.kEncoderNormalization);
      m_brAbsEncoderPub.set(m_backRight.getAbsoluteEncoderRaw() / SwerveConstants.kEncoderNormalization);
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
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
    return Math.IEEEremainder(m_gyro.getAngle(IMUAxis.kY), SwerveConstants.kGyroWrapModulo);
  }

  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
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
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(SwerveConstants.kXFormationAngleFrontLeft)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(SwerveConstants.kXFormationAngleFrontRight)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(SwerveConstants.kXFormationAngleBackLeft)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(SwerveConstants.kXFormationAngleBackRight)));
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
  }
}

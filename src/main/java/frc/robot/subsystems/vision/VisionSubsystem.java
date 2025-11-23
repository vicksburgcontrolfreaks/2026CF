// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {
  private final NetworkTable m_limelightTable;
  private final SwerveDrive m_swerveDrive;

  private boolean m_hasTarget = false;
  private double m_latency = 0.0;
  private Pose3d m_botPose3d = new Pose3d();
  private double m_tagCount = 0;
  private double m_tagSpan = 0;
  private double m_avgTagDist = 0;
  private double m_avgTagArea = 0;

  public VisionSubsystem(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
    m_limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName);
  }

  @Override
  public void periodic() {
    updateVisionData();

    // Update pose estimation if we have a valid target
    if (m_hasTarget && shouldAcceptVisionMeasurement()) {
      Pose2d visionPose2d = m_botPose3d.toPose2d();
      double timestamp = Timer.getFPGATimestamp() - (m_latency / 1000.0);

      // Add vision measurement with custom standard deviations based on distance and tag count
      m_swerveDrive.addVisionMeasurement(
        visionPose2d,
        timestamp,
        getEstimationStdDevs()
      );
    }

    // Publish telemetry
    SmartDashboard.putBoolean("Vision/Has Target", m_hasTarget);
    SmartDashboard.putNumber("Vision/Tag Count", m_tagCount);
    SmartDashboard.putNumber("Vision/Avg Distance", m_avgTagDist);
    SmartDashboard.putNumber("Vision/Latency", m_latency);
    SmartDashboard.putString("Vision/Bot Pose", m_botPose3d.toString());
  }

  private void updateVisionData() {
    // Read botpose data from Limelight
    double[] botPoseArray = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);

    if (botPoseArray.length >= 7) {
      m_botPose3d = new Pose3d(
        botPoseArray[0],
        botPoseArray[1],
        botPoseArray[2],
        new edu.wpi.first.math.geometry.Rotation3d(
          Math.toRadians(botPoseArray[3]),
          Math.toRadians(botPoseArray[4]),
          Math.toRadians(botPoseArray[5])
        )
      );
      m_latency = botPoseArray[6];
    }

    // Read additional data
    m_hasTarget = m_limelightTable.getEntry("tv").getDouble(0) > 0.5;
    m_tagCount = m_limelightTable.getEntry("botpose_tagcount").getDouble(0);
    m_tagSpan = m_limelightTable.getEntry("botpose_span").getDouble(0);
    m_avgTagDist = m_limelightTable.getEntry("botpose_avgdist").getDouble(0);
    m_avgTagArea = m_limelightTable.getEntry("botpose_avgarea").getDouble(0);
  }

  private boolean shouldAcceptVisionMeasurement() {
    // Don't accept if we don't have any tags
    if (m_tagCount < 1) {
      return false;
    }

    // Don't accept if tags are too far away
    if (m_avgTagDist > VisionConstants.kMaxDistanceMeters) {
      return false;
    }

    // Don't accept if pose ambiguity is too high (when available)
    // Note: Limelight v4 provides this via botpose_ambiguity
    double ambiguity = m_limelightTable.getEntry("botpose_ambiguity").getDouble(0);
    if (ambiguity > VisionConstants.kMaxPoseAmbiguity) {
      return false;
    }

    return true;
  }

  private edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> getEstimationStdDevs() {
    // Adjust trust based on number of tags and distance
    // More tags and closer distance = more trust (lower std dev)

    double xyStdDev = VisionConstants.kVisionStdDevs[0];
    double rotStdDev = VisionConstants.kVisionStdDevs[2];

    // Increase standard deviation based on distance
    if (m_avgTagDist > 2.0) {
      xyStdDev *= (m_avgTagDist / 2.0);
    }

    // Decrease standard deviation if we see multiple tags
    if (m_tagCount >= 2) {
      xyStdDev /= Math.sqrt(m_tagCount);
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
  }

  public boolean hasTarget() {
    return m_hasTarget;
  }

  public Pose3d getBotPose3d() {
    return m_botPose3d;
  }

  public Pose2d getBotPose2d() {
    return m_botPose3d.toPose2d();
  }

  public double getTagCount() {
    return m_tagCount;
  }

  public double getAverageTagDistance() {
    return m_avgTagDist;
  }

  public void setLEDMode(int mode) {
    // 0 = default, 1 = off, 2 = blink, 3 = on
    m_limelightTable.getEntry("ledMode").setNumber(mode);
  }

  public void setPipeline(int pipeline) {
    m_limelightTable.getEntry("pipeline").setNumber(pipeline);
  }
}

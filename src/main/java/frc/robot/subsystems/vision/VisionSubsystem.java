// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * VisionSubsystem using Limelight 4 with MegaTag2 for AprilTag-based localization.
 * MegaTag2 provides multi-tag pose estimation for improved accuracy and robustness.
 */
public class VisionSubsystem extends SubsystemBase {
  private final SwerveDrive m_swerveDrive;
  private final String m_limelightName;

  private PoseEstimate m_lastEstimate;
  private boolean m_hasTarget = false;

  public VisionSubsystem(SwerveDrive swerveDrive) {
    this(swerveDrive, VisionConstants.kLimelightName);
  }

  public VisionSubsystem(SwerveDrive swerveDrive, String limelightName) {
    m_swerveDrive = swerveDrive;
    m_limelightName = limelightName;
  }

  @Override
  public void periodic() {
    // Get MegaTag2 pose estimate using LimelightHelpers
    // botpose_wpiblue returns pose in WPILib blue alliance coordinate system
    m_lastEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);

    // Check if we have a valid target
    m_hasTarget = LimelightHelpers.getTV(m_limelightName);

    // Update pose estimation if we have a valid measurement
    if (m_hasTarget && shouldAcceptVisionMeasurement()) {
      Pose2d visionPose2d = m_lastEstimate.pose;
      double timestamp = m_lastEstimate.timestampSeconds;

      // Add vision measurement with dynamic standard deviations
      m_swerveDrive.addVisionMeasurement(
        visionPose2d,
        timestamp,
        getEstimationStdDevs()
      );
    }

    // Publish telemetry
    updateTelemetry();
  }

  /**
   * Determines whether to accept the current vision measurement based on quality metrics.
   * MegaTag2 provides robust filtering, but we add additional checks for safety.
   */
  private boolean shouldAcceptVisionMeasurement() {
    if (m_lastEstimate == null) {
      return false;
    }

    // Don't accept if we don't have any tags
    if (m_lastEstimate.tagCount < 1) {
      return false;
    }

    // Don't accept if tags are too far away
    if (m_lastEstimate.avgTagDist > VisionConstants.kMaxDistanceMeters) {
      return false;
    }

    // Don't accept if average tag area is too small (tags are far or small in frame)
    if (m_lastEstimate.avgTagArea < VisionConstants.kMinTagArea) {
      return false;
    }

    // MegaTag2 handles ambiguity internally and provides robust filtering
    // The pose estimate already accounts for multi-tag geometry and outlier rejection

    return true;
  }

  /**
   * Calculate dynamic standard deviations based on distance and number of tags.
   * More tags and closer distances = higher confidence (lower std dev).
   * This is key for MegaTag2 to work well with pose estimation.
   */
  private edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> getEstimationStdDevs() {
    if (m_lastEstimate == null) {
      return VecBuilder.fill(
        VisionConstants.kVisionStdDevs[0],
        VisionConstants.kVisionStdDevs[1],
        VisionConstants.kVisionStdDevs[2]
      );
    }

    double xyStdDev = VisionConstants.kSingleTagStdDevs[0];
    double rotStdDev = VisionConstants.kSingleTagStdDevs[2];

    // Multi-tag measurements are much more reliable
    if (m_lastEstimate.tagCount >= 2) {
      xyStdDev = VisionConstants.kMultiTagStdDevs[0];
      rotStdDev = VisionConstants.kMultiTagStdDevs[2];
    }

    // Increase standard deviation based on average distance to tags
    // Further tags = less confidence
    double avgDist = m_lastEstimate.avgTagDist;
    if (avgDist > 2.0) {
      double distanceFactor = avgDist / 2.0;
      xyStdDev *= distanceFactor;
      rotStdDev *= distanceFactor;
    }

    // MegaTag2 specific: use tag span and area to further refine confidence
    // Larger tag span (tags spread across frame) = better geometry = more confidence
    if (m_lastEstimate.tagCount >= 2) {
      double tagSpan = m_lastEstimate.tagSpan;
      if (tagSpan < VisionConstants.kMinTagSpan) {
        // Tags too close together, increase uncertainty
        xyStdDev *= 2.0;
      }
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
  }

  private void updateTelemetry() {
    SmartDashboard.putBoolean("Vision/Has Target", m_hasTarget);

    if (m_lastEstimate != null) {
      SmartDashboard.putNumber("Vision/Tag Count", m_lastEstimate.tagCount);
      SmartDashboard.putNumber("Vision/Avg Distance", m_lastEstimate.avgTagDist);
      SmartDashboard.putNumber("Vision/Avg Area", m_lastEstimate.avgTagArea);
      SmartDashboard.putNumber("Vision/Tag Span", m_lastEstimate.tagSpan);
      SmartDashboard.putNumber("Vision/Latency", m_lastEstimate.latency);
      SmartDashboard.putString("Vision/Bot Pose", m_lastEstimate.pose.toString());

      // Useful for debugging
      SmartDashboard.putNumber("Vision/Raw FID Count", m_lastEstimate.rawFiducials.length);
    }
  }

  // Public getters for commands/other subsystems

  public boolean hasTarget() {
    return m_hasTarget;
  }

  public PoseEstimate getLastEstimate() {
    return m_lastEstimate;
  }

  public Pose2d getBotPose2d() {
    return m_lastEstimate != null ? m_lastEstimate.pose : new Pose2d();
  }

  public double getTagCount() {
    return m_lastEstimate != null ? m_lastEstimate.tagCount : 0;
  }

  public double getAverageTagDistance() {
    return m_lastEstimate != null ? m_lastEstimate.avgTagDist : 0.0;
  }

  /**
   * Set LED mode on the Limelight.
   * @param mode 0 = pipeline default, 1 = off, 2 = blink, 3 = on
   */
  public void setLEDMode(int mode) {
    LimelightHelpers.setLEDMode_ForceOff(m_limelightName);
    if (mode == 3) {
      LimelightHelpers.setLEDMode_ForceOn(m_limelightName);
    } else if (mode == 2) {
      LimelightHelpers.setLEDMode_ForceBlink(m_limelightName);
    } else if (mode == 0) {
      LimelightHelpers.setLEDMode_PipelineControl(m_limelightName);
    }
  }

  /**
   * Switch to a different pipeline.
   * @param pipeline Pipeline index (0-9)
   */
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(m_limelightName, pipeline);
  }

  /**
   * Get the currently active pipeline index.
   */
  public int getPipeline() {
    return (int) LimelightHelpers.getCurrentPipelineIndex(m_limelightName);
  }

  /**
   * Take a snapshot with the current timestamp.
   */
  public void takeSnapshot() {
    LimelightHelpers.takeSnapshot(m_limelightName, "snapshot");
  }
}

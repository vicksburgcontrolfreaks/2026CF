// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * PhotonVisionSubsystem using 4x Arducam OV9281 cameras via PhotonVision for AprilTag-based localization.
 * Supports multi-camera pose estimation for 360° coverage and improved accuracy.
 */
public class PhotonVisionSubsystem extends SubsystemBase {
  // Camera data class to hold camera info
  private static class CameraData {
    public final PhotonCamera camera;
    public final PhotonPoseEstimator poseEstimator;
    public final String name;
    public PhotonPipelineResult lastResult;

    public CameraData(String cameraName, Transform3d robotToCamera, AprilTagFieldLayout fieldLayout) {
      this.name = cameraName;
      this.camera = new PhotonCamera(cameraName);
      this.poseEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCamera
      );
      this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      this.lastResult = null;
    }
  }

  private final SwerveDrive m_swerveDrive;
  private final List<CameraData> m_cameras = new ArrayList<>();
  private final AprilTagFieldLayout m_fieldLayout;

  // Telemetry publishers
  private final NetworkTable m_telemetryTable;
  private final BooleanPublisher m_hasTargetPub;
  private final DoublePublisher m_totalTagCountPub;
  private final DoublePublisher m_activeCamerasPub;
  private final StringPublisher m_bestCameraPub;
  private final StringPublisher m_rejectionReasonPub;
  private final BooleanPublisher m_measurementAcceptedPub;
  private final DoublePublisher m_latencyPub;

  // Per-camera publishers
  private final List<BooleanPublisher> m_cameraHasTargetPubs = new ArrayList<>();
  private final List<DoublePublisher> m_cameraTagCountPubs = new ArrayList<>();

  public PhotonVisionSubsystem(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;

    // Load AprilTag field layout (2025 Reefscape)
    try {
      m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTag field layout!", e.getStackTrace());
      throw new RuntimeException("Failed to load AprilTag field layout", e);
    }

    // Initialize all 4 cameras
    initializeCamera(
      PhotonVisionConstants.kCameraFrontName,
      PhotonVisionConstants.kRobotToFrontCamera
    );
    initializeCamera(
      PhotonVisionConstants.kCameraBackName,
      PhotonVisionConstants.kRobotToBackCamera
    );
    initializeCamera(
      PhotonVisionConstants.kCameraLeftName,
      PhotonVisionConstants.kRobotToLeftCamera
    );
    initializeCamera(
      PhotonVisionConstants.kCameraRightName,
      PhotonVisionConstants.kRobotToRightCamera
    );

    // Initialize NetworkTables publishers
    m_telemetryTable = NetworkTableInstance.getDefault().getTable("PhotonVision");
    m_hasTargetPub = m_telemetryTable.getBooleanTopic("Has Target").publish();
    m_totalTagCountPub = m_telemetryTable.getDoubleTopic("Total Tag Count").publish();
    m_activeCamerasPub = m_telemetryTable.getDoubleTopic("Active Cameras").publish();
    m_bestCameraPub = m_telemetryTable.getStringTopic("Best Camera").publish();
    m_rejectionReasonPub = m_telemetryTable.getStringTopic("Rejection Reason").publish();
    m_measurementAcceptedPub = m_telemetryTable.getBooleanTopic("Measurement Accepted").publish();
    m_latencyPub = m_telemetryTable.getDoubleTopic("Latency").publish();

    // Initialize per-camera publishers
    for (CameraData camData : m_cameras) {
      m_cameraHasTargetPubs.add(
        m_telemetryTable.getBooleanTopic(camData.name + "/Has Target").publish()
      );
      m_cameraTagCountPubs.add(
        m_telemetryTable.getDoubleTopic(camData.name + "/Tag Count").publish()
      );
    }
  }

  /**
   * Initialize a camera and add it to the camera list
   */
  private void initializeCamera(String cameraName, Transform3d robotToCamera) {
    try {
      CameraData camData = new CameraData(cameraName, robotToCamera, m_fieldLayout);
      m_cameras.add(camData);
      System.out.println("PhotonVision: Initialized camera: " + cameraName);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to initialize camera: " + cameraName, e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    boolean hasAnyTarget = false;
    int totalTagCount = 0;
    int activeCameras = 0;
    String bestCamera = "None";
    double bestConfidence = 0.0;
    EstimatedRobotPose bestPose = null;

    // Update all cameras and find best pose
    for (int i = 0; i < m_cameras.size(); i++) {
      CameraData camData = m_cameras.get(i);

      // Get latest result
      camData.lastResult = camData.camera.getLatestResult();

      boolean hasTarget = camData.lastResult.hasTargets();
      int tagCount = hasTarget ? camData.lastResult.getTargets().size() : 0;

      // Update per-camera telemetry
      m_cameraHasTargetPubs.get(i).set(hasTarget);
      m_cameraTagCountPubs.get(i).set(tagCount);

      if (hasTarget) {
        hasAnyTarget = true;
        totalTagCount += tagCount;
        activeCameras++;

        // Set reference pose for the estimator (uses current odometry pose)
        camData.poseEstimator.setReferencePose(m_swerveDrive.getPose());

        // Get pose estimate from this camera
        Optional<EstimatedRobotPose> poseResult = camData.poseEstimator.update(camData.lastResult);

        if (poseResult.isPresent()) {
          EstimatedRobotPose pose = poseResult.get();

          // Calculate confidence based on number of tags and ambiguity
          double confidence = calculatePoseConfidence(pose, camData.lastResult);

          if (confidence > bestConfidence) {
            bestConfidence = confidence;
            bestPose = pose;
            bestCamera = camData.name;
          }
        }
      }
    }

    // Update overall telemetry
    m_hasTargetPub.set(hasAnyTarget);
    m_totalTagCountPub.set(totalTagCount);
    m_activeCamerasPub.set(activeCameras);
    m_bestCameraPub.set(bestCamera);

    // Process best pose if available
    if (bestPose != null) {
      String rejectionReason = getVisionRejectionReason(bestPose);
      boolean accepted = rejectionReason == null;

      if (accepted) {
        // Add vision measurement to pose estimator
        m_swerveDrive.addVisionMeasurement(
          bestPose.estimatedPose.toPose2d(),
          bestPose.timestampSeconds,
          getEstimationStdDevs(bestPose)
        );

        m_rejectionReasonPub.set("None - Accepted");
        m_latencyPub.set(bestPose.timestampSeconds);
      } else {
        m_rejectionReasonPub.set(rejectionReason);
      }

      m_measurementAcceptedPub.set(accepted);
    } else {
      m_rejectionReasonPub.set(hasAnyTarget ? "No valid pose estimate" : "No targets");
      m_measurementAcceptedPub.set(false);
    }
  }

  /**
   * Calculate confidence score for a pose estimate
   * Higher score = more confident
   */
  private double calculatePoseConfidence(EstimatedRobotPose pose, PhotonPipelineResult result) {
    double confidence = 0.0;

    // More tags = higher confidence
    int tagCount = pose.targetsUsed.size();
    confidence += tagCount * 10.0;

    // Lower ambiguity = higher confidence
    if (result.hasTargets() && result.getBestTarget() != null) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      double ambiguity = bestTarget.getPoseAmbiguity();

      // Ambiguity ranges from 0.0 (perfect) to 1.0 (ambiguous)
      // Invert it so lower ambiguity gives higher confidence
      if (ambiguity < PhotonVisionConstants.kMaxAmbiguity) {
        confidence += (1.0 - ambiguity) * 5.0;
      }
    }

    return confidence;
  }

  /**
   * Determines whether to accept the current vision measurement based on quality metrics.
   * Returns null if measurement should be accepted, or a rejection reason string if it should be rejected.
   */
  private String getVisionRejectionReason(EstimatedRobotPose pose) {
    if (pose == null) {
      return "Null pose estimate";
    }

    // Don't accept if we don't have any tags
    if (pose.targetsUsed.isEmpty()) {
      return "No tags used in estimate";
    }

    // Check tag distance - reject if any tag is too far
    for (PhotonTrackedTarget target : pose.targetsUsed) {
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      if (distance > PhotonVisionConstants.kMaxTagDistance) {
        return String.format("Tag %d too far (%.2fm > %.2fm)",
          target.getFiducialId(), distance, PhotonVisionConstants.kMaxTagDistance);
      }
    }

    // Check pose ambiguity if available
    if (pose.targetsUsed.size() == 1) {
      PhotonTrackedTarget target = pose.targetsUsed.get(0);
      double ambiguity = target.getPoseAmbiguity();

      if (ambiguity > PhotonVisionConstants.kMaxAmbiguity) {
        return String.format("Pose ambiguity too high (%.3f > %.3f)",
          ambiguity, PhotonVisionConstants.kMaxAmbiguity);
      }
    }

    // Alliance color validation - ensure pose is reasonable for our alliance
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      double poseX = pose.estimatedPose.getX();

      // FRC field is ~16.54m long. Blue alliance starts at X=0, Red alliance at X=16.54
      // Reject if robot appears to be on wrong side of field
      if (alliance.get() == Alliance.Blue && poseX > 12.0) {
        return String.format("Blue alliance robot on red side (X=%.2fm)", poseX);
      } else if (alliance.get() == Alliance.Red && poseX < 4.54) {
        return String.format("Red alliance robot on blue side (X=%.2fm)", poseX);
      }
    }

    return null; // Accept measurement
  }

  /**
   * Calculate dynamic standard deviations based on distance and number of tags.
   * More tags and closer distances = higher confidence (lower std dev).
   */
  private edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> getEstimationStdDevs(
      EstimatedRobotPose pose) {

    double xyStdDev = PhotonVisionConstants.kSingleTagStdDevs[0];
    double rotStdDev = PhotonVisionConstants.kSingleTagStdDevs[2];

    // Multi-tag measurements are much more reliable
    if (pose.targetsUsed.size() >= PhotonVisionConstants.kMinTagsForHighConfidence) {
      xyStdDev = PhotonVisionConstants.kMultiTagStdDevs[0];
      rotStdDev = PhotonVisionConstants.kMultiTagStdDevs[2];
    }

    // Calculate average distance to tags
    double avgDistance = 0.0;
    for (PhotonTrackedTarget target : pose.targetsUsed) {
      avgDistance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    avgDistance /= pose.targetsUsed.size();

    // Increase standard deviation based on average distance to tags
    // Further tags = less confidence
    if (avgDistance > 2.0) {
      double distanceFactor = avgDistance / 2.0;
      xyStdDev *= distanceFactor;
      rotStdDev *= distanceFactor;
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
  }

  // Public getters for commands/other subsystems

  /**
   * Check if any camera has a target
   */
  public boolean hasTarget() {
    for (CameraData camData : m_cameras) {
      if (camData.lastResult != null && camData.lastResult.hasTargets()) {
        return true;
      }
    }
    return false;
  }

  /**
   * Get total number of tags seen across all cameras
   */
  public int getTotalTagCount() {
    int count = 0;
    for (CameraData camData : m_cameras) {
      if (camData.lastResult != null && camData.lastResult.hasTargets()) {
        count += camData.lastResult.getTargets().size();
      }
    }
    return count;
  }

  /**
   * Get number of active cameras (cameras with targets)
   */
  public int getActiveCameraCount() {
    int count = 0;
    for (CameraData camData : m_cameras) {
      if (camData.lastResult != null && camData.lastResult.hasTargets()) {
        count++;
      }
    }
    return count;
  }

  /**
   * Get result from a specific camera by name
   */
  public PhotonPipelineResult getCameraResult(String cameraName) {
    for (CameraData camData : m_cameras) {
      if (camData.name.equals(cameraName)) {
        return camData.lastResult;
      }
    }
    return null;
  }

  /**
   * Get all camera names
   */
  public List<String> getCameraNames() {
    List<String> names = new ArrayList<>();
    for (CameraData camData : m_cameras) {
      names.add(camData.name);
    }
    return names;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

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

  private final SwerveDriveSubsystem m_swerveDrive;
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

  public PhotonVisionSubsystem(SwerveDriveSubsystem swerveDrive) {
    m_swerveDrive = swerveDrive;

    // Load AprilTag field layout (2025 Reefscape)
    try {
      m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
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
    m_telemetryTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kTelemetryTableName);
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

    // Track all valid poses for fusion
    List<WeightedPose> validPoses = new ArrayList<>();

    // Update all cameras and collect valid poses
    for (int i = 0; i < m_cameras.size(); i++) {
      CameraData camData = m_cameras.get(i);

      // Get latest result (using newer getAllUnreadResults and taking the last one)
      var results = camData.camera.getAllUnreadResults();
      if (!results.isEmpty()) {
        camData.lastResult = results.get(results.size() - 1);
      } else {
        // No new results, keep using the last result
        continue;
      }

      boolean hasTarget = camData.lastResult.hasTargets();
      int tagCount = hasTarget ? camData.lastResult.getTargets().size() : 0;

      // Update per-camera telemetry
      m_cameraHasTargetPubs.get(i).set(hasTarget);
      m_cameraTagCountPubs.get(i).set(tagCount);

      // OPTIMIZATION: Skip pose estimation if no tags detected
      if (!hasTarget) {
        continue;
      }

      hasAnyTarget = true;
      totalTagCount += tagCount;
      activeCameras++;

      // Set reference pose for the estimator (uses current odometry pose)
      camData.poseEstimator.setReferencePose(m_swerveDrive.getPose());

      // Get pose estimate from this camera
      Optional<EstimatedRobotPose> poseResult = camData.poseEstimator.update(camData.lastResult);

      if (poseResult.isPresent()) {
        EstimatedRobotPose pose = poseResult.get();

        // Check if pose passes quality filters
        String rejectionReason = getVisionRejectionReason(pose);
        if (rejectionReason == null) {
          // Calculate confidence for this pose
          double confidence = calculatePoseConfidence(pose, camData.lastResult);

          // Add to valid poses list for fusion
          validPoses.add(new WeightedPose(pose, confidence, camData.name));
        }
      }
    }

    // Update overall telemetry
    m_hasTargetPub.set(hasAnyTarget);
    m_totalTagCountPub.set(totalTagCount);
    m_activeCamerasPub.set(activeCameras);

    // MULTI-CAMERA POSE FUSION: Fuse all valid poses
    if (!validPoses.isEmpty()) {
      FusedPoseResult fusedResult = fusePoses(validPoses);

      m_bestCameraPub.set(fusedResult.primaryCamera);
      m_rejectionReasonPub.set("None - Accepted (Fused from " + validPoses.size() + " camera(s))");
      m_measurementAcceptedPub.set(true);
      m_latencyPub.set(fusedResult.timestamp);

      // Add fused measurement to pose estimator
      m_swerveDrive.addVisionMeasurement(
        fusedResult.fusedPose,
        fusedResult.timestamp,
        fusedResult.stdDevs
      );
    } else {
      m_bestCameraPub.set("None");
      m_rejectionReasonPub.set(hasAnyTarget ? "No valid pose estimate" : "No targets");
      m_measurementAcceptedPub.set(false);
    }
  }

  /**
   * Helper class to store a pose estimate with its confidence weight and source camera
   */
  private static class WeightedPose {
    public final EstimatedRobotPose pose;
    public final double weight;
    public final String cameraName;

    public WeightedPose(EstimatedRobotPose pose, double weight, String cameraName) {
      this.pose = pose;
      this.weight = weight;
      this.cameraName = cameraName;
    }
  }

  /**
   * Result of pose fusion containing the fused pose and metadata
   */
  private static class FusedPoseResult {
    public final Pose2d fusedPose;
    public final double timestamp;
    public final edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> stdDevs;
    public final String primaryCamera;

    public FusedPoseResult(
        Pose2d fusedPose,
        double timestamp,
        edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> stdDevs,
        String primaryCamera) {
      this.fusedPose = fusedPose;
      this.timestamp = timestamp;
      this.stdDevs = stdDevs;
      this.primaryCamera = primaryCamera;
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
    confidence += tagCount * VisionConstants.kConfidenceTagCountMultiplier;

    // Lower ambiguity = higher confidence
    if (result.hasTargets() && result.getBestTarget() != null) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      double ambiguity = bestTarget.getPoseAmbiguity();

      // Ambiguity ranges from 0.0 (perfect) to 1.0 (ambiguous)
      // Invert it so lower ambiguity gives higher confidence
      if (ambiguity < PhotonVisionConstants.kMaxAmbiguity) {
        confidence += (1.0 - ambiguity) * VisionConstants.kConfidenceAmbiguityMultiplier;
      }
    }

    return confidence;
  }

  /**
   * Fuse multiple camera pose estimates into a single weighted pose.
   * Uses confidence-weighted averaging for position and heading.
   *
   * @param poses List of weighted poses from different cameras
   * @return Fused pose result with combined position, timestamp, and trust
   */
  private FusedPoseResult fusePoses(List<WeightedPose> poses) {
    if (poses.isEmpty()) {
      throw new IllegalArgumentException("Cannot fuse empty pose list");
    }

    // If only one pose, return it directly (optimized path)
    if (poses.size() == 1) {
      WeightedPose single = poses.get(0);
      return new FusedPoseResult(
        single.pose.estimatedPose.toPose2d(),
        single.pose.timestampSeconds,
        getEstimationStdDevs(single.pose),
        single.cameraName
      );
    }

    // Calculate total weight for normalization
    double totalWeight = 0.0;
    for (WeightedPose wp : poses) {
      totalWeight += wp.weight;
    }

    // Weighted average of X and Y positions
    double fusedX = 0.0;
    double fusedY = 0.0;

    // For rotation, we need to handle angle wrapping properly
    // Convert to unit vectors and average
    double cosSum = 0.0;
    double sinSum = 0.0;

    // Track highest confidence pose for reference
    WeightedPose bestPose = poses.get(0);
    double bestWeight = poses.get(0).weight;

    // Track latest timestamp
    double latestTimestamp = poses.get(0).pose.timestampSeconds;

    for (WeightedPose wp : poses) {
      double normalizedWeight = wp.weight / totalWeight;

      // Weighted position
      Pose2d pose2d = wp.pose.estimatedPose.toPose2d();
      fusedX += pose2d.getX() * normalizedWeight;
      fusedY += pose2d.getY() * normalizedWeight;

      // Weighted rotation (using vector averaging to handle wrapping)
      double angle = pose2d.getRotation().getRadians();
      cosSum += Math.cos(angle) * normalizedWeight;
      sinSum += Math.sin(angle) * normalizedWeight;

      // Track best pose
      if (wp.weight > bestWeight) {
        bestWeight = wp.weight;
        bestPose = wp;
      }

      // Use latest timestamp
      if (wp.pose.timestampSeconds > latestTimestamp) {
        latestTimestamp = wp.pose.timestampSeconds;
      }
    }

    // Construct fused rotation from averaged vectors
    Rotation2d fusedRotation = new Rotation2d(cosSum, sinSum);

    // Construct fused pose
    Pose2d fusedPose = new Pose2d(new Translation2d(fusedX, fusedY), fusedRotation);

    // Calculate fused standard deviations (lower StdDev because multiple cameras agree)
    // Use the best pose as baseline, but reduce StdDev based on number of cameras
    edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> baseStdDevs =
      getEstimationStdDevs(bestPose.pose);

    // Reduce uncertainty based on number of cameras (more cameras = more confidence)
    // Use factor of 1/sqrt(n) which is statistically sound for independent measurements
    double reductionFactor = 1.0 / Math.sqrt(poses.size());

    edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> fusedStdDevs =
      VecBuilder.fill(
        baseStdDevs.get(0, 0) * reductionFactor,
        baseStdDevs.get(1, 0) * reductionFactor,
        baseStdDevs.get(2, 0) * reductionFactor
      );

    return new FusedPoseResult(fusedPose, latestTimestamp, fusedStdDevs, bestPose.cameraName);
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
      // Reject if robot appears to be on wrong side of field (beyond midfield)
      if (alliance.get() == Alliance.Blue && poseX < VisionConstants.kBlueAllianceXThreshold) {
        return String.format("Blue alliance robot on red side (X=%.2fm)", poseX);
      } else if (alliance.get() == Alliance.Red && poseX > VisionConstants.kRedAllianceXThreshold) {
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
    if (avgDistance > VisionConstants.kDistanceFactorThreshold) {
      double distanceFactor = avgDistance / VisionConstants.kDistanceFactorThreshold;
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

  /**
   * Get all unique AprilTag IDs currently detected across all cameras
   * @return List of unique AprilTag IDs (fiducial IDs)
   */
  public List<Integer> getDetectedAprilTagIDs() {
    List<Integer> tagIDs = new ArrayList<>();

    for (CameraData camData : m_cameras) {
      if (camData.lastResult != null && camData.lastResult.hasTargets()) {
        for (PhotonTrackedTarget target : camData.lastResult.getTargets()) {
          int tagID = target.getFiducialId();
          // Only add if not already in list (avoid duplicates from multiple cameras)
          if (!tagIDs.contains(tagID)) {
            tagIDs.add(tagID);
          }
        }
      }
    }

    return tagIDs;
  }
}

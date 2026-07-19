// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.curtinfrc.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator poseEstimator;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    poseEstimator = new PhotonPoseEstimator(Vision.aprilTagLayout, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> poseEstimate = poseEstimator.estimateCoprocMultiTagPose(result);
      poseEstimate = poseEstimator.estimateCoprocMultiTagPose(result);
      if (poseObservations.isEmpty()) {
        poseEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
      }

      if (poseEstimate.isPresent()) {
        EstimatedRobotPose estimate = poseEstimate.get(); // Convert from optional if present
        for (var target : result.targets) { // Get tagIds used
          tagIds.add((short) target.fiducialId);
        }

        double totalTagDistance = 0.0; // Calculate average tag distance
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        double ambiguity =
            result.multitagResult.isPresent() // Calculate ambiguity
                ? result.multitagResult.get().estimatedPose.ambiguity
                : result.targets.get(0).poseAmbiguity;

        poseObservations.add( // Add observation
            new PoseObservation(
                estimate.timestampSeconds,
                estimate.estimatedPose,
                ambiguity,
                tagIds.size(),
                totalTagDistance / result.targets.size(),
                PoseObservationType.PHOTONVISION)); // Observation type
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}

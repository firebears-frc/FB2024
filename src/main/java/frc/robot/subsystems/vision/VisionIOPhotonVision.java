package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonVision implements VisionIO {
    public static final class Constants {
        public static final String CAMERA_NAME = "Arducam_OV2311_USB_Camera";
    }

    private final PhotonCamera visionSystem = new PhotonCamera(Constants.CAMERA_NAME);
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(Vision.Constants.FIELD_LAYOUT, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.Constants.CAMERA_OFFSET);

    public VisionIOPhotonVision() {
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Initialize inputs
        inputs.hasTargets = false;
        inputs.targetIDs = new int[0];
        inputs.targetAmbiguities = new double[0];
        inputs.targetTransforms = new Transform3d[0];
        inputs.hasPose = false;
        inputs.pose = null;
        inputs.timestamp = 0;
        inputs.strategy = PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
        inputs.usedTargetIDs = new int[0];

        // Check if the system is connected
        inputs.connected = visionSystem.isConnected();
        if (!inputs.connected)
            return;

        // Get the result of the vision system
        PhotonPipelineResult pipelineResult = visionSystem.getLatestResult();
        inputs.hasTargets = pipelineResult.hasTargets();
        if (!inputs.hasTargets)
            return;

        // Convert the targets to logged types
        inputs.targetIDs = pipelineResult.targets.stream().mapToInt(target -> target.getFiducialId()).toArray();
        inputs.targetAmbiguities = pipelineResult.targets.stream().mapToDouble(target -> target.getPoseAmbiguity()).toArray();
        inputs.targetTransforms = pipelineResult.targets.stream().map(target -> target.getBestCameraToTarget()).toArray(Transform3d[]::new);

        // Get the pose result
        Optional<EstimatedRobotPose> poseResult = poseEstimator.update(pipelineResult);
        inputs.hasPose = poseResult.isPresent();
        if (!inputs.hasPose)
            return;

        // Convert the pose result to logged types
        EstimatedRobotPose result = poseResult.get();
        inputs.pose = result.estimatedPose;
        inputs.timestamp = result.timestampSeconds;
        inputs.strategy = result.strategy;
        inputs.usedTargetIDs = result.targetsUsed.stream().mapToInt(target -> target.getFiducialId()).toArray();
    }
}

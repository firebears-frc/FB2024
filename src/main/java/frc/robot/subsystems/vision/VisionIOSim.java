package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOSim implements VisionIO {
    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim = new VisionSystemSim("");

    private final PhotonCamera visionSystem = new PhotonCamera("");
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(Vision.Constants.FIELD_LAYOUT, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.Constants.CAMERA_OFFSET);

    public VisionIOSim(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        var properties = new SimCameraProperties();
        properties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        properties.setCalibError(0.35, 0.10);
        properties.setFPS(15);
        properties.setAvgLatencyMs(50);
        properties.setLatencyStdDevMs(15);

        cameraSim = new PhotonCameraSim(visionSystem, properties);

        visionSim.addAprilTags(Vision.Constants.FIELD_LAYOUT);
        visionSim.addCamera(cameraSim, Vision.Constants.CAMERA_OFFSET);

        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d pose = poseSupplier.get();
        visionSim.update(pose);

        // Initialize inputs
        inputs.connected = true;
        inputs.hasTargets = false;
        inputs.targetIDs = new int[0];
        inputs.targetAmbiguities = new double[0];
        inputs.targetTransforms = new Transform3d[0];
        inputs.hasPose = false;
        inputs.pose = null;
        inputs.timestamp = 0;
        inputs.strategy = PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
        inputs.usedTargetIDs = new int[0];

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
}

package com.seiford.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Vision {
    private static final class Constants {
        public static final String NAME = "Arducam_OV2311_USB_Camera";
        public static final Transform3d CAMERA_TRANSFORM = new Transform3d(new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0)),
                new Rotation3d());
    }

    private final PhotonCamera visionSystem;
    private final PhotonPoseEstimator poseEstimator;
    private final BiConsumer<Pose2d, Double> consumer;

    public Vision(BiConsumer<Pose2d, Double> consumer) {
        visionSystem = new PhotonCamera(Constants.NAME);
        this.consumer = consumer;

        AprilTagFieldLayout fieldLayout = null;
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Failed to load AprilTag layout!");
            e.printStackTrace();
            poseEstimator = null;
            return;
        }

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                visionSystem,
                Constants.CAMERA_TRANSFORM);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void periodic() {
        Logger.recordOutput("Drive/Localization/Vision/IsConnected", visionSystem.isConnected());
        if (!visionSystem.isConnected())
            return;

        PhotonPipelineResult pipelineResult = visionSystem.getLatestResult();
        Logger.recordOutput("Drive/Localization/Vision/HasTargets", pipelineResult.hasTargets());
        if (!pipelineResult.hasTargets())
            return;

        MultiTargetPNPResult multiResult = pipelineResult.getMultiTagResult();
        Logger.recordOutput("Drive/Localization/Vision/MultiTargetPresent", multiResult.estimatedPose.isPresent);
        Logger.recordOutput("Drive/Localization/Vision/MultiTargetCount", multiResult.fiducialIDsUsed.size());
        Logger.recordOutput("Drive/Localization/Vision/MultiTargets",
                multiResult.fiducialIDsUsed.stream().mapToInt(target -> target).toArray());

        Logger.recordOutput("Drive/Localization/Vision/TargetCount", pipelineResult.targets.size());
        Logger.recordOutput("Drive/Localization/Vision/Targets",
                pipelineResult.targets.stream().mapToInt(target -> target.getFiducialId()).toArray());

        List<PhotonTrackedTarget> badTargets = new ArrayList<>();
        for (PhotonTrackedTarget target : pipelineResult.targets) {
            if (target.getPoseAmbiguity() > 0.2)
                badTargets.add(target);
        }
        pipelineResult.targets.removeAll(badTargets);
        Logger.recordOutput("Drive/Localization/Vision/BadTargetCount", badTargets.size());
        Logger.recordOutput("Drive/Localization/Vision/BadTargets",
                badTargets.stream().mapToInt(target -> target.getFiducialId()).toArray());

        Logger.recordOutput("Drive/Localization/Vision/GoodTargetCount", pipelineResult.targets.size());
        Logger.recordOutput("Drive/Localization/Vision/GoodTargets",
                pipelineResult.targets.stream().mapToInt(target -> target.getFiducialId()).toArray());

        if (poseEstimator == null)
            return;

        Optional<EstimatedRobotPose> poseResult = poseEstimator.update(pipelineResult);
        Logger.recordOutput("Drive/Localization/Vision/PosePresent", poseResult.isPresent());
        if (!poseResult.isPresent())
            return;

        EstimatedRobotPose result = poseResult.get();
        Logger.recordOutput("Drive/Localization/Vision/Pose", result.estimatedPose);
        Logger.recordOutput("Drive/Localization/Vision/Timestamp", result.timestampSeconds);
        Logger.recordOutput("Drive/Localization/Vision/Strategy", result.strategy);
        Logger.recordOutput("Drive/Localization/Vision/TargetCount", result.targetsUsed.size());
        Logger.recordOutput("Drive/Localization/Vision/Targets",
                result.targetsUsed.stream().mapToInt(target -> target.getFiducialId()).toArray());

        if (consumer != null)
            consumer.accept(result.estimatedPose.toPose2d(), result.timestampSeconds);
    }
}

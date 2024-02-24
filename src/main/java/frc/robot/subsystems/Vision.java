package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private final PhotonCamera photonCamera;    
    private final PhotonPoseEstimator poseEstimator;
    private final BiConsumer<Pose2d, Double> consumer;

    public Vision(BiConsumer<Pose2d, Double> consumer) throws IOException{
        photonCamera = new PhotonCamera(Constants.VisionConstants.kCameraName);
        poseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.VisionConstants.kCameraOffset);
        this.consumer = consumer;
    }

    @Override
    public void periodic(){
        boolean connected = photonCamera.isConnected();
        Logger.recordOutput("Vision/Connected", connected);
        if (!connected)
            return;

        PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
        boolean hasTargets = pipelineResult.hasTargets();
        Logger.recordOutput("Vision/HasTargets", hasTargets);
        if (!hasTargets)
            return;

        List<PhotonTrackedTarget> badTargets = new ArrayList<>();
        for(PhotonTrackedTarget target : pipelineResult.targets){
            if(target.getPoseAmbiguity()>0.5){
                badTargets.add(target);
            }
        }

        pipelineResult.targets.removeAll(badTargets);
        Logger.recordOutput("Vision/badTargets", badTargets.size());
        
        Optional<EstimatedRobotPose> poseResult = poseEstimator.update(pipelineResult);
        boolean posePresent = poseResult.isPresent();
        Logger.recordOutput("Vision/HasPose", posePresent);
        if (!posePresent)
            return;

        EstimatedRobotPose estimatedPose = poseResult.get();
        Logger.recordOutput("Vision/Pose", estimatedPose.estimatedPose);
        Logger.recordOutput("Vision/Timestamp", estimatedPose.timestampSeconds);
        Logger.recordOutput("Vision/Targets", estimatedPose.targetsUsed.size());
        Logger.recordOutput("Vision/Strategy", estimatedPose.strategy);

        consumer.accept(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    }
}

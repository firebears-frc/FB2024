package frc.robot.drive;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Vision {
    private static final class Constants {
        public static final String NAME = "MainC";
        public static final Transform3d CAMERA_TRANSFORM = new Transform3d(new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0)),
                new Rotation3d());
    }

    private final UsbCamera driverCamera;
    private final PhotonCamera visionSystem;
    private final PhotonPoseEstimator poseEstimator;
    private final BiConsumer<Pose2d, Double> consumer;

    @AutoLogOutput(key = "Drive/Localization/Vision/Status")
    private Status status;

    public Vision(BiConsumer<Pose2d, Double> consumer) {
        driverCamera = CameraServer.startAutomaticCapture();
        driverCamera.setResolution(320, 240);

        visionSystem = new PhotonCamera(Constants.NAME);
        this.consumer = consumer;

        status = Status.NOT_CONNECTED;

        AprilTagFieldLayout fieldLayout = null;
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
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

    private enum Status {
        NOT_CONNECTED,
        NO_TARGETS,
        NO_POSE_ESTIMATOR,
        NO_POSE_RESULT,
        NO_CONSUMER,
        POSE_FOUND
    }

    private Status getPhotonUpdate() {
        boolean connected = visionSystem.isConnected();
        if (!connected) {
            return Status.NOT_CONNECTED;
        }

        PhotonPipelineResult pipelineResult = visionSystem.getLatestResult();
        if (!pipelineResult.hasTargets()) {
            return Status.NO_TARGETS;
        }

        if (poseEstimator == null) {
            return Status.NO_POSE_ESTIMATOR;
        }

        Optional<EstimatedRobotPose> poseResult = poseEstimator.update();
        if (!poseResult.isPresent()) {
            return Status.NO_POSE_RESULT;
        }

        EstimatedRobotPose result = poseResult.get();
        if (consumer == null) {
            return Status.NO_CONSUMER;
        }

        consumer.accept(result.estimatedPose.toPose2d(), result.timestampSeconds);
        Logger.recordOutput("Drive/Localization/Vision/Pose", result.estimatedPose);
        Logger.recordOutput("Drive/Localization/Vision/Timestamp", result.timestampSeconds);
        Logger.recordOutput("Drive/Localization/Vision/Strategy", result.strategy);
        return Status.POSE_FOUND;
    }

    public void periodic() {
        status = getPhotonUpdate();
    }
}

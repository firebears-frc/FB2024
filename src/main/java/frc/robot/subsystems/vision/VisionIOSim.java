package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionIOSim implements VisionIO {
    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim = new VisionSystemSim("");

    private final PhotonCamera visionSystem = new PhotonCamera("");

    public VisionIOSim(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        var properties = new SimCameraProperties();
        properties.setCalibration(1280, 800, Rotation2d.fromDegrees(75));
        properties.setCalibError(0.35, 0.10);
        properties.setFPS(15);
        properties.setAvgLatencyMs(50);
        properties.setLatencyStdDevMs(15);

        cameraSim = new PhotonCameraSim(visionSystem, properties);

        visionSim.addAprilTags(Vision.Constants.FIELD_LAYOUT);
        visionSim.addCamera(cameraSim, Vision.Constants.CAMERA_OFFSET);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d pose = poseSupplier.get();
        visionSim.update(pose);

        inputs.connected = true;
        inputs.pipelineResult = visionSystem.getLatestResult();
    }
}

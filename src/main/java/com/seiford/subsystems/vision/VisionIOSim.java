package com.seiford.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOSim implements VisionIO {
  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;
  private final PhotonCamera visionSystem;

  public VisionIOSim(Supplier<Pose2d> poseSupplier, Transform3d cameraOffset, String name, SimCameraProperties properties) {
    this.poseSupplier = poseSupplier;

    visionSim = new VisionSystemSim(name + "_sim");
    visionSystem = new PhotonCamera(name + "_cam");
    cameraSim = new PhotonCameraSim(visionSystem, properties);

    visionSim.addAprilTags(Vision.Constants.FIELD_LAYOUT);
    visionSim.addCamera(cameraSim, cameraOffset);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Pose2d pose = poseSupplier.get();
    visionSim.update(pose);

    inputs.connected = true;
    inputs.pipelineResult = visionSystem.getLatestResult();
  }

  public static SimCameraProperties arducamOV2311() {
    var properties = new SimCameraProperties();
    properties.setCalibration(1280, 800, Rotation2d.fromDegrees(75));
    properties.setCalibError(0.35, 0.10);
    properties.setFPS(25);
    properties.setAvgLatencyMs(25);
    properties.setLatencyStdDevMs(10);
    return properties;
  }
}

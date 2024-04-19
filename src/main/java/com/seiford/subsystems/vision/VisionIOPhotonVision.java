package com.seiford.subsystems.vision;

import org.photonvision.PhotonCamera;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera visionSystem;

  public VisionIOPhotonVision(String cameraName) {
    visionSystem = new PhotonCamera(cameraName);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = visionSystem.isConnected();
    if (inputs.connected) {
      inputs.pipelineResult = visionSystem.getLatestResult();
    }
  }
}

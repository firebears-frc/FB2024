package com.seiford.subsystems.vision;

import org.photonvision.PhotonCamera;

public class VisionIOPhotonVision implements VisionIO {
  public static final class Constants {
    public static final String CAMERA_NAME = "Arducam_OV2311_USB_Camera";
  }

  private final PhotonCamera visionSystem = new PhotonCamera(Constants.CAMERA_NAME);

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = visionSystem.isConnected();
    if (inputs.connected) {
      inputs.pipelineResult = visionSystem.getLatestResult();
    }
  }
}

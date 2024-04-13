package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected;

    public boolean hasTargets;
    public int[] targetIDs;
    public double[] targetAmbiguities;
    public Transform3d[] targetTransforms;

    public boolean hasPose;
    public Pose3d pose;
    public double timestamp;
    public PhotonPoseEstimator.PoseStrategy strategy;
    public int[] usedTargetIDs;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {
  }
}

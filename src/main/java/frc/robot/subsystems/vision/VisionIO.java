package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;

    public boolean hasTargets = false;
    public int[] targetIDs = new int[0];
    public double[] targetAmbiguities = new double[0];
    public Transform3d[] targetTransforms = new Transform3d[0];

    public boolean hasPose = false;
    public Pose3d pose = new Pose3d();
    public double timestamp = 0.0;
    public PhotonPoseEstimator.PoseStrategy strategy = PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
    public int[] usedTargetIDs = new int[0];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {
  }
}

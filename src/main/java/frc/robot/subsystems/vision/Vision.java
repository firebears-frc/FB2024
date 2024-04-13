package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.VisionData;

public class Vision extends SubsystemBase {
  public static final class Constants {
    public static final Transform3d CAMERA_OFFSET = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-11.5),
            Units.inchesToMeters(0),
            Units.inchesToMeters(13.00)),
        new Rotation3d(
            Rotation2d.fromDegrees(180).getRadians(),
            Rotation2d.fromDegrees(-30.0).getRadians(),
            Rotation2d.fromDegrees(180).getRadians()));
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);
  }
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(Constants.FIELD_LAYOUT, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.Constants.CAMERA_OFFSET);
  private final Consumer<VisionData> consumer;

  public Vision(VisionIO io, Consumer<VisionData> consumer) {
    this.io = io;
    this.consumer = consumer;

    LogTable.disableProtobufWarning(); // TODO: This may cause loop overruns
    poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
  }

  private Matrix<N3, N1> getStdDevs(List<PhotonTrackedTarget> targets, Pose3d estimatedPose) {
      var stdDevs = Constants.SINGLE_TAG_STD_DEVS;

      int targetCount = 0;
      double distance = 0;
      for (var target : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
          if (tagPose.isEmpty()) {
              continue;
          }

          targetCount++;
          distance += tagPose.get().getTranslation().getDistance(estimatedPose.getTranslation());
      }

      // If we have no targets, exit early
      if (targetCount == 0) {
          return stdDevs;
      }

      // Convert to average distance
      distance /= targetCount;

      // Decrease std devs if multiple targets are visible
      if (targetCount > 1) {
          stdDevs = Constants.MULTI_TAG_STD_DEVS;
      }

      // Increase std devs based on (average) distance
      if (targetCount == 1.0 && distance > 4.0) {
          stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      } else {
          stdDevs = stdDevs.times(1 + (distance * distance / 30.0));
      }

      return stdDevs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    if (!inputs.connected || !inputs.pipelineResult.hasTargets())
      return;

    Logger.recordOutput("Vision/TargetIDs", inputs.pipelineResult.targets.stream().mapToInt(target -> target.getFiducialId()).toArray());
    Logger.recordOutput("Vision/TargetAmbiguities", inputs.pipelineResult.targets.stream().mapToDouble(target -> target.getPoseAmbiguity()).toArray());

    Optional<EstimatedRobotPose> poseResult = poseEstimator.update(inputs.pipelineResult);
    Logger.recordOutput("Vision/HasPose", poseResult.isPresent());
    if (!poseResult.isPresent())
      return;

    EstimatedRobotPose result = poseResult.get();
    Logger.recordOutput("Vision/Pose", result.estimatedPose);
    Logger.recordOutput("Vision/Timestamp", result.timestampSeconds);
    Logger.recordOutput("Vision/Strategy", result.strategy);
    Logger.recordOutput("Vision/TargetIDsUsed", result.targetsUsed.stream().mapToInt(target -> target.getFiducialId()).toArray());

    Matrix<N3, N1> stdDevs = getStdDevs(inputs.pipelineResult.targets, result.estimatedPose);
    Logger.recordOutput("Vision/StdDevs", stdDevs.transpose().getData());

    consumer.accept(new VisionData(result.estimatedPose, result.timestampSeconds, stdDevs));
  }
}

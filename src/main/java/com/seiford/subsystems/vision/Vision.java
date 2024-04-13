package com.seiford.subsystems.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.seiford.util.VisionData;

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
import com.seiford.subsystems.vision.VisionIOInputsAutoLogged;

public class Vision extends SubsystemBase {
  public static final class Constants {
    public static final Transform3d CAMERA_OFFSET = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-11.50),
            Units.inchesToMeters(0.00),
            Units.inchesToMeters(13.00)),
        new Rotation3d(
            Rotation2d.fromDegrees(180.0).getRadians(),
            Rotation2d.fromDegrees(-30.0).getRadians(),
            Rotation2d.fromDegrees(180.0).getRadians()));
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.00, 4.00, 8.00);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.50, 0.50, 1.00);
    public static final Matrix<N3, N1> INVALID_STD_DEVS = VecBuilder.fill(0.0, 0.0, 0.0);
  }

  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(Constants.FIELD_LAYOUT,
      PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.Constants.CAMERA_OFFSET);
  private final Consumer<VisionData> consumer;

  public Vision(VisionIO io, Consumer<VisionData> consumer) {
    this.io = io;
    this.consumer = consumer;

    LogTable.disableProtobufWarning(); // TODO: This may cause loop overruns
    poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
  }

  private Matrix<N3, N1> getStdDevs(List<PhotonTrackedTarget> targets, Pose3d estimatedPose) {
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
      return null;
    }

    // Convert to average distance
    distance /= targetCount;

    // Decrease std devs if multiple targets are visible
    Matrix<N3, N1> stdDevs;
    if (targetCount > 1) {
      stdDevs = Constants.MULTI_TAG_STD_DEVS;
    } else {
      stdDevs = Constants.SINGLE_TAG_STD_DEVS;
    }

    // Increase std devs based on (average) distance
    if (targetCount == 1.0 && distance > 4.0) {
      stdDevs = null;
    } else {
      stdDevs = stdDevs.times(1 + (distance * distance / 30.0));
    }

    return stdDevs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    if (!inputs.connected) {
      Logger.recordOutput("Vision/TargetIDs", new int[0]);
      Logger.recordOutput("Vision/HasPose", false);
      Logger.recordOutput("Vision/Pose", new Pose3d());
      Logger.recordOutput("Vision/Strategy", "No Connection");
      Logger.recordOutput("Vision/StdDevs", new double[0]);
      Logger.recordOutput("Vision/UsedTargetPoses", new Pose3d[0]);
      Logger.recordOutput("Vision/TargetIDsUsed", new int[0]);
      return;
    }

    if (!inputs.pipelineResult.hasTargets()) {
      Logger.recordOutput("Vision/TargetIDs", new int[0]);
      Logger.recordOutput("Vision/HasPose", false);
      Logger.recordOutput("Vision/Pose", new Pose3d());
      Logger.recordOutput("Vision/Strategy", "No Targets");
      Logger.recordOutput("Vision/StdDevs", new double[0]);
      Logger.recordOutput("Vision/UsedTargetPoses", new Pose3d[0]);
      Logger.recordOutput("Vision/TargetIDsUsed", new int[0]);
      return;
    }

    Logger.recordOutput("Vision/TargetIDs", inputs.pipelineResult.targets.stream().mapToInt(target -> target.getFiducialId()).toArray());

    Optional<EstimatedRobotPose> poseResult = poseEstimator.update(inputs.pipelineResult);
    Logger.recordOutput("Vision/HasPose", poseResult.isPresent());
    if (!poseResult.isPresent()) {
      Logger.recordOutput("Vision/Pose", new Pose3d());
      Logger.recordOutput("Vision/Strategy", "No Pose");
      Logger.recordOutput("Vision/StdDevs", new double[0]);
      Logger.recordOutput("Vision/UsedTargetPoses", new Pose3d[0]);
      Logger.recordOutput("Vision/TargetIDsUsed", new int[0]);
      return;
    }

    EstimatedRobotPose result = poseResult.get();

    Matrix<N3, N1> stdDevs = getStdDevs(inputs.pipelineResult.targets, result.estimatedPose);
    if (stdDevs == null) {
      Logger.recordOutput("Vision/Pose", new Pose3d());
      Logger.recordOutput("Vision/Strategy", "Invalid Standard Deviations");
      Logger.recordOutput("Vision/StdDevs", new double[0]);
      Logger.recordOutput("Vision/UsedTargetPoses", new Pose3d[0]);
      Logger.recordOutput("Vision/TargetIDsUsed", new int[0]);
      return;
    }

    Logger.recordOutput("Vision/Pose", result.estimatedPose);
    Logger.recordOutput("Vision/Strategy", result.strategy);
    Logger.recordOutput("Vision/StdDevs", stdDevs.transpose().getData());
    Logger.recordOutput("Vision/UsedTargetPoses", result.targetsUsed.stream().map(
        target -> result.estimatedPose.transformBy(Constants.CAMERA_OFFSET).transformBy(target.getBestCameraToTarget()))
        .toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/TargetIDsUsed",
        result.targetsUsed.stream().mapToInt(target -> target.getFiducialId()).toArray());

    consumer.accept(new VisionData(result.estimatedPose, result.timestampSeconds, stdDevs));
  }
}
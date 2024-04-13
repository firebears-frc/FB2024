package frc.robot.util;

import edu.wpi.first.math.Matrix;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionData {
  public final Pose3d pose;
  public final double timestamp;
  public final Matrix<N3, N1> stdDevs;

  public VisionData(Pose3d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    this.pose = pose;
    this.timestamp = timestamp;
    this.stdDevs = stdDevs;
  }
}

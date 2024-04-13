package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
}

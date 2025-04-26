package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static final class ArmConstants {
    public static final double shoulderP = 0.02;
    public static final double shoulderI = 0.0;
    public static final double shoulderD = 0.0;
    public static final double shoulderG = 0.35;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
  // visionconstants
  public static final class VisionConstants {
    public static final String kCameraName = "Arducam_OV2311_USB_Camera";
    public static final Transform3d kCameraOffset =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.5), Units.inchesToMeters(0), Units.inchesToMeters(13.00)),
            new Rotation3d(
                Rotation2d.fromDegrees(180).getRadians(),
                Rotation2d.fromDegrees(-30.0).getRadians(),
                Rotation2d.fromDegrees(180).getRadians()));
  }
}

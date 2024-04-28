package com.seiford.subsystems.conductor;

import com.seiford.util.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Conductor extends SubsystemBase {
  private static final class Constants {
    public static final Translation2d BLUE_SPEAKER = new Translation2d(0.0, 5.550);
    public static final Translation2d RED_SPEAKER = new Translation2d(16.541, 5.550);
  }

  private final List<Shot> shots =
      List.of(
          new Shot("Subwoofer", 1.29, 14.50, 2400.00), // Tuned subwoofer shot
          new Shot("Podium", 2.94, 20.00, 3200.00), // Not tuned podium shot
          new Shot("Bottom", 3.75, 33.75, 3300.00), // Partially tuned bottom side shot
          new Shot("Top", 4.68, 35.00, 3300.00), // Not tuned top side shot
          new Shot("Far", 6.60, 36.25, 3300.00) // Not tuned far shot
          );

  private final Supplier<Pose2d> poseSupplier;

  @AutoLogOutput(key = "Conductor/SpeakerTranslation")
  private Translation2d speakerTranslation;

  private Rotation2d armAngle;
  private double shooterRPM;

  public Conductor(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @AutoLogOutput(key = "Conductor/SpeakerDistance")
  private double getSpeakerDistance() {
    return Math.abs(speakerTranslation.getNorm());
  }

  @AutoLogOutput(key = "Conductor/SpeakerAngle")
  public Rotation2d getSpeakerAngle() {
    return speakerTranslation.getAngle();
  }

  @AutoLogOutput(key = "Conductor/ArmAngle")
  public Rotation2d getArmAngle() {
    return armAngle;
  }

  @AutoLogOutput(key = "Conductor/ShooterRPM")
  public double getShooterRPM() {
    return shooterRPM;
  }

  @AutoLogOutput(key = "Conductor/SpeakerPosition")
  private Translation2d getSpeakerPosition() {
    return Util.isRedAlliance() ? Constants.RED_SPEAKER : Constants.BLUE_SPEAKER;
  }

  @Override
  public void periodic() {
    speakerTranslation = poseSupplier.get().getTranslation().minus(getSpeakerPosition());

    Interpolator interpolator = new Interpolator();
    interpolator.putAll(shots);

    armAngle = interpolator.angle(getSpeakerDistance());
    shooterRPM = interpolator.speed(getSpeakerDistance());
  }

  private static final class Shot {
    private final LoggedDashboardNumber distance;
    private final LoggedDashboardNumber angle;
    private final LoggedDashboardNumber speed;

    public Shot(String name, double defaultDistance, double defaultAngle, double defaultSpeed) {
      distance = new LoggedDashboardNumber("Conductor/" + name + "/Distance", defaultDistance);
      angle = new LoggedDashboardNumber("Conductor/" + name + "/Angle", defaultAngle);
      speed = new LoggedDashboardNumber("Conductor/" + name + "/Speed", defaultSpeed);
    }

    public double distance() {
      return distance.get();
    }

    public double angle() {
      return angle.get();
    }

    public double speed() {
      return speed.get();
    }
  }

  private static final class Interpolator {
    private final InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap speedInterpolator = new InterpolatingDoubleTreeMap();

    public void putAll(List<Shot> shots) {
      for (Shot shot : shots) {
        angleInterpolator.put(shot.distance(), shot.angle());
        speedInterpolator.put(shot.distance(), shot.speed());
      }
    }

    public Rotation2d angle(double distance) {
      return Rotation2d.fromDegrees(angleInterpolator.get(distance));
    }

    public double speed(double distance) {
      return speedInterpolator.get(distance);
    }
  }
}

// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final class Constants {
    public static final Translation2d BLUE_SPEAKER = new Translation2d(0.00, 5.55);
    public static final Translation2d RED_SPEAKER = new Translation2d(16.58, 5.55);

    public static final Translation2d BLUE_AMP = new Translation2d(1.84, 8.20);
    public static final Translation2d RED_AMP = new Translation2d(14.70, 8.20);

    private static final double DEADBAND = 0.1;
  }

  private DriveCommands() {}

  private static Translation2d calculateLinearSpeeds(double x, double y, double linearSpeed) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), Constants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calcaulate new linear velocity
    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

    return linearVelocity.times(linearSpeed);
  }

  private static double calculateRotationSpeed(double omega, double rotationSpeed) {
    // Apply deadband
    omega = MathUtil.applyDeadband(omega, Constants.DEADBAND);

    // Square values
    omega = Math.copySign(omega * omega, omega);

    return omega * rotationSpeed;
  }

  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearSpeeds = calculateLinearSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), drive.getMaxLinearSpeedMetersPerSec());
          double rotationSpeed = calculateRotationSpeed(omegaSupplier.getAsDouble(), drive.getMaxAngularSpeedRadPerSec());

          // Convert to field relative speeds & send command
          drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(linearSpeeds.getX(), linearSpeeds.getY(), rotationSpeed,
              isRedAlliance() ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
        },
        drive);
  }

  private static Command targetLock(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> target) {

      ProfiledPIDController pid = new ProfiledPIDController(10.0, 0, 0,
          new TrapezoidProfile.Constraints(drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec() / 2));
      pid.enableContinuousInput(-Math.PI, Math.PI);
      return Commands.run(
          () -> {
            Translation2d linearSpeeds = calculateLinearSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), drive.getMaxLinearSpeedMetersPerSec());

            Translation2d targetTranslation = drive.getPose().getTranslation().minus(target.get());
            Rotation2d delta = drive.getPose().getRotation().minus(targetTranslation.getAngle());
            double rotationSpeed = pid.calculate(delta.getRadians());

            // Convert to field relative speeds & send command
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(linearSpeeds.getX(), linearSpeeds.getY(), rotationSpeed,
                isRedAlliance() ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
          },
          drive);
  }

  public static Command speakerLock(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return targetLock(drive, xSupplier, ySupplier,
        () -> isRedAlliance() ? Constants.RED_SPEAKER : Constants.BLUE_SPEAKER);
  }

  public static Command ampLock(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return targetLock(drive, xSupplier, ySupplier,
        () -> isRedAlliance() ? Constants.RED_AMP : Constants.BLUE_AMP);
  }
}

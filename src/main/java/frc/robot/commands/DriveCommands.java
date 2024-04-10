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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final class Constants {
    public static final Translation2d BLUE_SPEAKER = new Translation2d(0.0, 5.550);
    public static final Translation2d RED_SPEAKER = new Translation2d(16.541, 5.550);

    private static final double DEADBAND = 0.1;
  }

  private DriveCommands() {}

  private static ChassisSpeeds calculateSpeeds(double x, double y, double omega, double linearSpeed,
      double rotationSpeed) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), Constants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);
    omega = MathUtil.applyDeadband(omega, Constants.DEADBAND);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

    return new ChassisSpeeds(linearVelocity.getX() * linearSpeed, linearVelocity.getY() * linearSpeed,
        omega * rotationSpeed);
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
          ChassisSpeeds commanded = calculateSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
              omegaSupplier.getAsDouble(), drive.getMaxLinearSpeedMetersPerSec(), drive.getMaxAngularSpeedRadPerSec());

          // Convert to field relative speeds & send command
          drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(commanded,
              isRedAlliance() ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
        },
        drive);
  }

  private static Command targetLock(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> target) {
    try (PIDController pid = new PIDController(1.0, 0, 0)) {
      pid.enableContinuousInput(-Math.PI, Math.PI);
      return joystickDrive(drive, xSupplier, ySupplier, () -> {
        Translation2d targetTranslation = drive.getPose().getTranslation().minus(target.get());
        Logger.recordOutput("targetLock/targetAngle", targetTranslation.getAngle().getRadians());

        Logger.recordOutput("targetLock/driveAngle", drive.getPose().getRotation().getRadians());

        Rotation2d delta = drive.getPose().getRotation().minus(targetTranslation.getAngle());
        Logger.recordOutput("targetLock/deltaAngle", delta);

        double feedback = pid.calculate(delta.getRadians());
        Logger.recordOutput("targetLock/feedback", feedback);

        return feedback;
      });
    }
  }

  public static Command speakerLock(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return targetLock(drive, xSupplier, ySupplier,
        () -> isRedAlliance() ? Constants.RED_SPEAKER : Constants.BLUE_SPEAKER);
  }
}

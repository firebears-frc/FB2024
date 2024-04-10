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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class DriveCommands {
  private static final class Constants {
    public static final Translation2d BLUE_SPEAKER = new Translation2d(0.00, 5.55);
    public static final Translation2d RED_SPEAKER = new Translation2d(16.58, 5.55);

    public static final Translation2d BLUE_AMP = new Translation2d(1.84, 8.20);
    public static final Translation2d RED_AMP = new Translation2d(14.70, 8.20);

    public static final Pose2d BLUE_SUBWOOFER = new Pose2d(1.45, 5.55, Rotation2d.fromDegrees(0.0));

    public static final Pose2d BLUE_AMP_PLACEMENT = new Pose2d(1.84, 7.75, Rotation2d.fromDegrees(90.0));

    public static final Pose2d BLUE_SOURCE = new Pose2d(15.08, 1.00, Rotation2d.fromDegrees(0.0));

    private static final double DEADBAND = 0.1;
  }

  private DriveCommands() {
  }

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

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearSpeeds = calculateLinearSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
              Drive.Constants.MAX_LINEAR_SPEED);
          double rotationSpeed = calculateRotationSpeed(omegaSupplier.getAsDouble(), Drive.Constants.MAX_ANGULAR_SPEED);

          drive.runFieldVelocity(new ChassisSpeeds(linearSpeeds.getX(), linearSpeeds.getY(), rotationSpeed));
        },
        drive);
  }

  /**
   * Field relative drive command using one joysticks (controlling linear
   * velocities) and pointed at the target.
   */
  private static Command targetRotationLock(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> target) {

    ProfiledPIDController pid = new ProfiledPIDController(10.0, 0, 0,
        new TrapezoidProfile.Constraints(Drive.Constants.MAX_ANGULAR_SPEED, Drive.Constants.MAX_ANGULAR_ACCELERATION));
    pid.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
        () -> {
          Translation2d linearSpeeds = calculateLinearSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
              Drive.Constants.MAX_LINEAR_SPEED);

          Translation2d targetTranslation = drive.getPose().getTranslation().minus(target.get());
          Rotation2d delta = drive.getPose().getRotation().minus(targetTranslation.getAngle());
          double rotationSpeed = pid.calculate(delta.getRadians());

          drive.runFieldVelocity(new ChassisSpeeds(linearSpeeds.getX(), linearSpeeds.getY(), rotationSpeed));
        },
        drive);
  }

  /**
   * Field relative drive command using one joysticks (controlling linear
   * velocities) and pointed at the speaker.
   */
  public static Command speakerRotationLock(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return targetRotationLock(drive, xSupplier, ySupplier,
        () -> Util.isRedAlliance() ? Constants.RED_SPEAKER : Constants.BLUE_SPEAKER);
  }

  /**
   * Field relative drive command using one joysticks (controlling linear
   * velocities) and pointed at the amp.
   */
  public static Command ampRotationLock(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return targetRotationLock(drive, xSupplier, ySupplier,
        () -> Util.isRedAlliance() ? Constants.RED_AMP : Constants.BLUE_AMP);
  }

  private static Command followPath(Pose2d endPose) {
    return AutoBuilder.pathfindToPoseFlipped(
        endPose,
        new PathConstraints(
            Drive.Constants.MAX_LINEAR_SPEED, Drive.Constants.MAX_LINEAR_ACCELERATION,
            Drive.Constants.MAX_ANGULAR_SPEED, Drive.Constants.MAX_ANGULAR_ACCELERATION));
  }

  public static Command pathfindSpeaker() {
    return followPath(Constants.BLUE_SUBWOOFER);
  }

  public static Command pathfindAmp() {
    return followPath(Constants.BLUE_AMP_PLACEMENT);
  }

  public static Command pathfindSource() {
    return followPath(Constants.BLUE_SOURCE);
  }
}

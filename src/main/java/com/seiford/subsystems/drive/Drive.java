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

package com.seiford.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.seiford.util.LocalADStarAK;
import com.seiford.util.Util;
import com.seiford.util.VisionData;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.seiford.subsystems.drive.GyroIOInputsAutoLogged;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class Drive extends SubsystemBase {
  public static final class Constants {
    // Drive base size
    public static final double ROBOT_WIDTH = Units.inchesToMeters(22.0);
    public static final double ROBOT_LENGTH = Units.inchesToMeters(28.0);
    public static final double WHEEL_OFFSET = Units.inchesToMeters(1.75);
    public static final double TRACK_WIDTH_X = ROBOT_WIDTH - (2 * WHEEL_OFFSET);
    public static final double TRACK_WIDTH_Y = ROBOT_LENGTH - (2 * WHEEL_OFFSET);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
        new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
        new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
        new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };

    // Drive base limits
    public static final double MAX_LINEAR_SPEED = 7.8; // meters per second
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final double MAX_LINEAR_ACCELERATION = 4.5; // meters per second squared
    public static final double MAX_ANGULAR_ACCELERATION = MAX_LINEAR_ACCELERATION / DRIVE_BASE_RADIUS;

    // Orbit positions
    public static final Translation2d BLUE_SPEAKER = new Translation2d(0.00, 5.55);
    public static final Translation2d RED_SPEAKER = new Translation2d(16.58, 5.55);
    public static final Translation2d BLUE_AMP = new Translation2d(1.84, 8.20);
    public static final Translation2d RED_AMP = new Translation2d(14.70, 8.20);

    // Pathfind positions
    public static final Pose2d BLUE_SUBWOOFER = new Pose2d(1.45, 5.55, Rotation2d.fromDegrees(0.0));
    public static final Pose2d BLUE_AMP_PLACEMENT = new Pose2d(1.84, 7.75, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d BLUE_SOURCE = new Pose2d(15.08, 1.00, Rotation2d.fromDegrees(0.0));
    public static final Pose2d BLUE_STAGE = new Pose2d(5.85, 4.10, Rotation2d.fromDegrees(0.0));

    // Gamepad deadband TODO
    private static final double DEADBAND = 0.1;
  };

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.MODULE_TRANSLATIONS);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
      lastModulePositions, new Pose2d());
  private SwerveDrivePoseEstimator fusedVision = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
      lastModulePositions, new Pose2d());
  private final LoggedDashboardBoolean useVision = new LoggedDashboardBoolean("Localization/UseVision", true);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        this::getRobotVelocity,
        this::runRobotVelocity,
        new HolonomicPathFollowerConfig(Constants.MAX_LINEAR_SPEED, Constants.DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        Util::isRedAlliance,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Localization/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Localization/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              for (int i = 0; i < 4; i++) {
                modules[i].runCharacterization(voltage.in(Volts));
              }
            },
            null,
            this));
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.yawPosition;
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      odometry.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      fusedVision.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all of the
   * modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current robot pose. */
  @AutoLogOutput(key = "Localization/Robot")
  public Pose2d getPose() {
    return useVision.get() ? fusedVision.getEstimatedPosition() : odometry.getEstimatedPosition();
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Localization/Odometry")
  private Pose2d getOdometryPose() {
    return odometry.getEstimatedPosition();
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Localization/Vision")
  private Pose2d getVisionPose() {
    return fusedVision.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  private Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the current robot velocities. */
  @AutoLogOutput(key = "Localization/Velocity")
  private ChassisSpeeds getRobotVelocity() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Runs the drive at the desired field relative velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  private void runFieldVelocity(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
        Util.isRedAlliance() ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation());
    runRobotVelocity(speeds);
  }

  /**
   * Runs the drive at the desired robot relative velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  private void runRobotVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  private void stopDrive() {
    runRobotVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  private void stopDriveWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = Constants.MODULE_TRANSLATIONS[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stopDrive();
  }

  /** Resets the current odometry pose. */
  private void setPose(Pose2d pose) {
    odometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
    fusedVision.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param data.pose      The pose of the robot as measured by the vision camera.
   * @param data.timestamp The timestamp of the vision measurement in seconds.
   * @param data.stdDevs   The standard deviations to use for this vision pose.
   */
  public void addVisionMeasurement(VisionData data) {
    fusedVision.addVisionMeasurement(data.pose.toPose2d(), data.timestamp, data.stdDevs);
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
  public Command joystickDrive(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return run(
        () -> {
          Translation2d linearSpeeds = calculateLinearSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
              Constants.MAX_LINEAR_SPEED);
          double rotationSpeed = calculateRotationSpeed(omegaSupplier.getAsDouble(), Constants.MAX_ANGULAR_SPEED);

          runFieldVelocity(new ChassisSpeeds(linearSpeeds.getX(), linearSpeeds.getY(), rotationSpeed));
        });
  }

  /** Command to put the drive in an X. */
  public Command turtle() {
    return startEnd(this::stopDriveWithX, this::stopDrive);
  }

  public Command zeroHeading() {
    return runOnce(() -> setPose(new Pose2d(getPose().getTranslation(), new Rotation2d()))).ignoringDisable(true);
  }

  /**
   * Field relative drive command using one joystick (controlling linear
   * velocities) and pointed at the target.
   */
  private Command orbit(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> target) {
    ProfiledPIDController pid = new ProfiledPIDController(10.0, 0, 0,
        new TrapezoidProfile.Constraints(Constants.MAX_ANGULAR_SPEED, Constants.MAX_ANGULAR_ACCELERATION));
    pid.enableContinuousInput(-Math.PI, Math.PI);
    return run(
        () -> {
          Translation2d linearSpeeds = calculateLinearSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
              Constants.MAX_LINEAR_SPEED);

          Translation2d targetTranslation = getPose().getTranslation().minus(target.get());
          Rotation2d delta = getPose().getRotation().minus(targetTranslation.getAngle());
          double rotationSpeed = pid.calculate(delta.getRadians());

          runFieldVelocity(new ChassisSpeeds(linearSpeeds.getX(), linearSpeeds.getY(), rotationSpeed));
        });
  }

  /**
   * Field relative drive command using one joysticks (controlling linear
   * velocities) and pointed at the speaker.
   */
  public Command orbitSpeaker(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return orbit(xSupplier, ySupplier, () -> Util.isRedAlliance() ? Constants.RED_SPEAKER : Constants.BLUE_SPEAKER);
  }

  /**
   * Field relative drive command using one joysticks (controlling linear
   * velocities) and pointed at the amp.
   */
  public Command orbitAmp(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return orbit(xSupplier, ySupplier, () -> Util.isRedAlliance() ? Constants.RED_AMP : Constants.BLUE_AMP);
  }

  /** Pathfind to the specified end pose */
  private Command pathfind(Pose2d endPose) {
    return AutoBuilder.pathfindToPoseFlipped(
        endPose,
        new PathConstraints(
            Constants.MAX_LINEAR_SPEED, Constants.MAX_LINEAR_ACCELERATION,
            Constants.MAX_ANGULAR_SPEED, Constants.MAX_ANGULAR_ACCELERATION));
  }

  /** Pathfind to the speaker */
  public Command pathfindSpeaker() {
    return pathfind(Constants.BLUE_SUBWOOFER);
  }

  /** Pathfind to the amp */
  public Command pathfindAmp() {
    return pathfind(Constants.BLUE_AMP_PLACEMENT);
  }

  /** Pathfind to the source */
  public Command pathfindSource() {
    return pathfind(Constants.BLUE_SOURCE);
  }

  /** Pathfind to the stage */
  public Command pathfindStage() {
    return pathfind(Constants.BLUE_STAGE);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}

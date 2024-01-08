package com.seiford.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ChargeStationStatus;

public class Drive extends SubsystemBase {
    private static final class Constants {
        public static final double MAX_AUTO_VELOCITY = 4.5; // meters per second

        public static final double XY_CONTROLLER_P = 5.0;
        public static final double R_CONTROLLER_P = 5.0;

        public static final HolonomicPathFollowerConfig CONFIG = new HolonomicPathFollowerConfig(
                new PIDConstants(XY_CONTROLLER_P, 0.0, 0.0),
                new PIDConstants(R_CONTROLLER_P, 0.0, 0.0),
                MAX_AUTO_VELOCITY,
                Chassis.WHEEL_RADIUS,
                new ReplanningConfig());

        public static final double BALANCE_ON_CHARGE_STATION_SPEED = 0.5; // meters per second
        public static final double DRIVE_ONTO_CHARGE_STATION_SPEED = 2.0; // meters per second
    }

    private final Chassis chassis;
    private final Localization localization;

    private ChargeStationStatus chargeStationStatus = ChargeStationStatus.NONE;

    public Drive() {
        chassis = new Chassis();
        localization = new Localization(chassis.getKinematics(), chassis.getModulePositions());

        AutoBuilder.configureHolonomic(
                localization::getPose,
                this::setPose,
                chassis::getSpeeds,
                chassis::driveRobotRelative,
                Constants.CONFIG,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent())
                        return alliance.get() == DriverStation.Alliance.Red;

                    return false;
                },
                this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("Drive/Path/TargetPose", pose));
        PathPlannerLogging.setLogActivePathCallback(poses -> {
            Logger.recordOutput("Drive/Path/Poses", poses);
            Logger.recordOutput("Drive/Path/Start", poses.get(0));
            Logger.recordOutput("Drive/Path/Middle", poses.get(poses.size() / 2));
            Logger.recordOutput("Drive/Path/End", poses.get(poses.size()));
        });
    }

    private void setPose(Pose2d pose) {
        localization.setPose(pose, chassis.getModulePositions());
    }

    @Override
    public void periodic() {
        localization.periodic(chassis.getModulePositions());
        updateChargeStationStatus();
    }

    public Command zeroHeading() {
        return runOnce(() -> {
            Pose2d pose = localization.getPose();
            pose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0.0));
            localization.setPose(pose, chassis.getModulePositions());
        });
    }

    public Command turtle() {
        return startEnd(chassis::setX, () -> {
        });
    }

    public Command defaultCommand(Supplier<ChassisSpeeds> commandSupplier, boolean slowMode) {
        return new DefaultCommand(commandSupplier,
                speeds -> chassis.driveFieldRelative(speeds, localization.getRawYaw()), slowMode, this);
    }

    public Command autoBalance() {
        return Commands.sequence(
                // Drive until we are at a high enough angle
                runOnce(() -> chassis
                        .driveRobotRelative(new ChassisSpeeds(Constants.DRIVE_ONTO_CHARGE_STATION_SPEED, 0.0, 0.0))),
                Commands.waitUntil(localization::isOnChargeStation),

                // Rock back and forth until it stops and is level
                run(() -> {
                    if (!localization.isNotPitching()) {
                        // Charge station is moving, stop!
                        chassis.setX();
                        return;
                    }

                    // Depending on what way the charge station is tipped, go to middle
                    ChassisSpeeds speeds = new ChassisSpeeds(Constants.BALANCE_ON_CHARGE_STATION_SPEED, 0.0, 0.0);
                    if (localization.getPitch().getRadians() < 0)
                        speeds.vxMetersPerSecond *= -1.0;
                    chassis.driveRobotRelative(speeds);
                }).until(() -> localization.isNotPitching() && localization.isLevel()));
    }

    @AutoLogOutput(key = "Drive/ChargeStationStatus")
    public ChargeStationStatus chargeStationStatus() {
        return chargeStationStatus;
    }

    private void updateChargeStationStatus() {
        switch (chargeStationStatus) {
            case ENGAGED:
                if (!localization.isLevel() || !localization.isNotPitching()) {
                    chargeStationStatus = ChargeStationStatus.DOCKED;
                }
                break;
            case DOCKED:
                if (localization.isLevel() && localization.isNotPitching()) {
                    chargeStationStatus = ChargeStationStatus.ENGAGED;
                }
                break;
            case NONE:
            default:
                if (localization.isOnChargeStation()) {
                    chargeStationStatus = ChargeStationStatus.DOCKED;
                }
                break;
        }
    }
}

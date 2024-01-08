package frc.robot.drive;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Chassis {
    public static final double ROBOT_WIDTH = Units.inchesToMeters(25);
    public static final double ROBOT_LENGTH = Units.inchesToMeters(34);
    public static final double MAX_VELOCITY = 4.8; // meters per second

    // Wheels are offset 1.75" into the modules
    private static final double WHEEL_OFFSET = Units.inchesToMeters(1.75);
    private static final double TRACK_WIDTH = ROBOT_WIDTH - (WHEEL_OFFSET * 2);
    private static final double WHEEL_BASE = ROBOT_LENGTH - (WHEEL_OFFSET * 2);

    public static final double WHEEL_RADIUS = Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);

    private static final class Constants {
        public static final SwerveModuleConfiguration MODULES[] = {
                new SwerveModuleConfiguration(26, 27, Rotation2d.fromDegrees(-90),
                        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), "Front Left"),
                new SwerveModuleConfiguration(21, 20, Rotation2d.fromDegrees(0),
                        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), "Front Right"),
                new SwerveModuleConfiguration(24, 25, Rotation2d.fromDegrees(180),
                        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), "Rear Left"),
                new SwerveModuleConfiguration(23, 22, Rotation2d.fromDegrees(90),
                        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), "Rear Right")
        };
    }

    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;

    public Chassis() {
        // Build up modules array
        modules = new SwerveModule[Constants.MODULES.length];
        for (int i = 0; i < Constants.MODULES.length; i++) {
            modules[i] = new SwerveModule(Constants.MODULES[i]);
        }

        // Build up position offset array for kinematics
        Translation2d positionOffsets[] = new Translation2d[Constants.MODULES.length];
        for (int i = 0; i < Constants.MODULES.length; i++) {
            positionOffsets[i] = Constants.MODULES[i].positionOffset;
        }
        kinematics = new SwerveDriveKinematics(positionOffsets);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @AutoLogOutput(key = "Drive/Chassis/ModulePositions")
    public SwerveModulePosition[] getModulePositions() {
        // Build up position array
        SwerveModulePosition result[] = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            result[i] = modules[i].getPosition();
        }
        return result;
    }

    @AutoLogOutput(key = "Drive/Chassis/ModuleStates")
    public SwerveModuleState[] getModuleStates() {
        // Build up state array
        SwerveModuleState result[] = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            result[i] = modules[i].getState();
        }
        return result;
    }

    @AutoLogOutput(key = "Drive/Chassis/Speeds")
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds chassisSpeeds, Rotation2d yaw) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, yaw));
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        swerveDrive(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    private void swerveDrive(SwerveModuleState states[]) {
        if (states.length != Constants.MODULES.length)
            throw new IllegalStateException(
                    "Swerve module count error: " + states.length + ", " + Constants.MODULES.length);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY);

        for (int i = 0; i < Constants.MODULES.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public void setX() {
        SwerveModuleState[] states = new SwerveModuleState[Constants.MODULES.length];
        for (int i = 0; i < Constants.MODULES.length; i++) {
            states[i] = new SwerveModuleState(0, Constants.MODULES[i].positionOffset.getAngle());
        }
        swerveDrive(states);
    }
}

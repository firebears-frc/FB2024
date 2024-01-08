package frc.robot.drive;

import org.littletonrobotics.junction.AutoLogOutput;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class Localization {
    private static final class Constants {
        public static final Rotation2d LEVEL_TOLERANCE = Rotation2d.fromDegrees(2.0);
        public static final Rotation2d ON_TOLERANCE = Rotation2d.fromDegrees(10.0);
        public static final Rotation2d PITCH_VELOCITY_MAX = Rotation2d.fromDegrees(0.2); // per cycle
    }

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Vision vision;

    private AHRS navX;

    // Charge Station
    private Rotation2d pitch, pitchVelocity;

    public Localization(SwerveDriveKinematics kinematics, SwerveModulePosition[] initiaModulePositions) {
        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
            navX = null;
            poseEstimator = null;
            vision = new Vision(null); // initialize driver vision
            return;
        }

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getRawYaw(),
                initiaModulePositions,
                new Pose2d());
        vision = new Vision(this::visionPose);

        pitch = Rotation2d.fromDegrees(0.0);
        pitchVelocity = Rotation2d.fromDegrees(0.0);
    }

    @AutoLogOutput(key = "Drive/Localization/Active")
    public boolean isActive() {
        return navX != null && poseEstimator != null;
    }

    @AutoLogOutput(key = "Drive/Localization/RawYaw")
    public Rotation2d getRawYaw() {
        if (!isActive())
            return Rotation2d.fromDegrees(0.0);

        return navX.getRotation2d();
    }

    public void visionPose(Pose2d pose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public void setPose(Pose2d pose, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(getRawYaw(), modulePositions, pose);
    }

    @AutoLogOutput(key = "Drive/Localization/Pose")
    public Pose2d getPose() {
        if (!isActive())
            return new Pose2d();

        return poseEstimator.getEstimatedPosition();
    }

    public void periodic(SwerveModulePosition[] modulePositions) {
        if (!isActive())
            return;

        poseEstimator.update(getRawYaw(), modulePositions);
        vision.periodic();

        // Update pitch velocity and charge station status
        Rotation2d currentPitch = Rotation2d.fromDegrees(navX.getPitch());
        pitchVelocity = currentPitch.minus(pitch);
        pitch = currentPitch;

    }

    @AutoLogOutput(key = "Drive/Localization/ChargeStation/Pitch")
    public Rotation2d getPitch() {
        return pitch;
    }

    @AutoLogOutput(key = "Drive/Localization/ChargeStation/PitchVelocity")
    private Rotation2d getPitchVelocity() {
        return pitchVelocity;
    }

    @AutoLogOutput(key = "Drive/Localization/ChargeStation/IsLevel")
    public boolean isLevel() {
        return Math.abs(pitch.getRadians()) < Constants.LEVEL_TOLERANCE.getRadians();
    }

    @AutoLogOutput(key = "Drive/Localization/ChargeStation/IsOnChargeStation")
    public boolean isOnChargeStation() {
        return Math.abs(pitch.getRadians()) > Constants.ON_TOLERANCE.getRadians();
    }

    @AutoLogOutput(key = "Drive/Localization/ChargeStation/IsNotPitching")
    public boolean isNotPitching() {
        return Math.abs(pitchVelocity.getRadians()) < Constants.PITCH_VELOCITY_MAX.getRadians();
    }
}

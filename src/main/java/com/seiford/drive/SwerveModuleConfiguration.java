package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConfiguration {
    protected final int drivingID;
    protected final int turningID;
    protected final Rotation2d angleOffset;
    protected final Translation2d positionOffset;
    protected final String name;

    public SwerveModuleConfiguration(int drivingID, int turningID, Rotation2d angleOffset, Translation2d positionOffset,
            String name) {
        this.drivingID = drivingID;
        this.turningID = turningID;
        this.angleOffset = angleOffset;
        this.positionOffset = positionOffset;
        this.name = name;
    }
}

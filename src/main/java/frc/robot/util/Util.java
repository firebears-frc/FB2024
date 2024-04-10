package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class Util {
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}

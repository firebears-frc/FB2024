package com.seiford;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Util {
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent())
            return alliance.get() == DriverStation.Alliance.Red;

        DriverStation.reportWarning("Unable to get alliance color", true);
        return false;
    }
}

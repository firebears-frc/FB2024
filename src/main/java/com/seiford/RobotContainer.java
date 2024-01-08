package com.seiford;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    private static final class Constants {
        public static final int PDH_CAN_ID = 1;
    }

    private final PowerDistribution pdh;

    public RobotContainer() {
        pdh = new PowerDistribution(Constants.PDH_CAN_ID, ModuleType.kRev);

        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

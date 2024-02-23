package com.seiford;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    /*public static class Constants {
        public static final Map<Integer, String> CAN_IDs = Map.of(
                1, "Front Left Driving",
                2, "Front Left Turning",
                3, "Front Right Driving",
                4, "Front Right Turning",
                5, "Rear Left Driving",
                6, "Rear Left Turning",
                7, "Rear Right Driving",
                8, "Rear Right Turning",
                9, "Intake",
                10, "Top Shooter",
                11, "Bottom Shooter",
                12, "Right Arm",
                13, "Left Arm",
                14, "Right Climber",
                15, "Left Climber");
    }*/

    private RobotContainer robotContainer;
    private Command autonomousCommand;

    @Override
    public void robotInit() {
        initializeLogging();

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    private void initializeLogging() {
        Logger.recordMetadata("Project Name", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("Branch Name", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("Commit Hash (Short)", BuildConstants.GIT_SHA.substring(0, 8));
        Logger.recordMetadata("Commit Hash (Full)", BuildConstants.GIT_SHA);
        Logger.recordMetadata("Build Time", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("Dirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        if (isReal()) {
            // Log to USB & Network Tables
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            // Replay from log and save to file
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        Logger.registerURCL(URCL.startExternal(/*Constants.CAN_IDs*/));
        Logger.start();
    }
}

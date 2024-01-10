// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

    private void initializeLogging() {
        Logger logger = Logger.getInstance();
        logger.recordMetadata("Project Name", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("Branch Name", BuildConstants.GIT_BRANCH);
        logger.recordMetadata("Commit Hash (Short)", BuildConstants.GIT_SHA.substring(0, 8));
        logger.recordMetadata("Commit Hash (Full)", BuildConstants.GIT_SHA);
        logger.recordMetadata("Build Time", BuildConstants.BUILD_DATE);

        if (isReal()) {
            // Log to USB & Network Tables
            logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
            logger.addDataReceiver(new NT4Publisher());
        } else {
            // Replay from log and save to file
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            logger.setReplaySource(new WPILOGReader(logPath));
            logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        logger.start();
    }
}
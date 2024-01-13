// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Bass;
import frc.robot.subsystems.DownBeat;

public class RobotContainer {
  private final Bass m_robotDrive = new Bass();
   private final DownBeat m_intake = new DownBeat();
  private final CommandJoystick one = new CommandJoystick(0);
  private final CommandJoystick two = new CommandJoystick(1);

  private final XboxController xboxController = new XboxController(2);

  public RobotContainer() {
    configureBindings();

      m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(one.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(one.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(two.getX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  private void configureBindings() {

    JoystickButton xboxXButton = new JoystickButton(xboxController, XboxController.Button.kX.value);
    xboxXButton.onTrue(new InstantCommand(m_intake::intakeNote, m_intake));

  }

  public XboxController getXboxController() {
    return xboxController;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }  
}

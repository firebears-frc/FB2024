// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Bass;

public class RobotContainer {
    private final Bass m_robotDrive = new Bass();
    private final CommandJoystick one = new CommandJoystick(0);
    private final CommandJoystick two = new CommandJoystick(1);
    private final LoggedDashboardChooser<Command> autoChooser;

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
      autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    }

    private void configureBindings() {
        one.trigger().toggleOnTrue(new StartEndCommand(m_robotDrive::setX, () -> {
        }, m_robotDrive));

        two.trigger().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
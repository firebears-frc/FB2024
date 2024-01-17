package com.seiford;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.seiford.drive.Drive;
import com.seiford.subsystems.Downbeat;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private static final class Constants {
        public static final int JOYSTICK_1_PORT = 0;
        public static final int JOYSTICK_2_PORT = 1;
        public static final int CONTROLLER_PORT = 2;

        public static final double JOYSTICK_DEADBAND = 0.05;

        public static final int PDH_CAN_ID = 1;
    }

    private final Drive drive;
    private final Downbeat intake;
    private final PowerDistribution pdh;

    private final CommandJoystick one;
    private final CommandJoystick two;
    private final CommandXboxController controller;
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        drive = new Drive();
        intake = new Downbeat();
        pdh = new PowerDistribution(Constants.PDH_CAN_ID, ModuleType.kRev);

        one = new CommandJoystick(Constants.JOYSTICK_1_PORT);
        two = new CommandJoystick(Constants.JOYSTICK_2_PORT);
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);

        configureAutoCommands();
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());

        configureBindings();
    }

    private void configureAutoCommands() {
        NamedCommands.registerCommands(Map.of(
                "TODO", Commands.none()));
    }

    private ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(
                -MathUtil.applyDeadband(one.getY(), Constants.JOYSTICK_DEADBAND),
                -MathUtil.applyDeadband(one.getX(), Constants.JOYSTICK_DEADBAND),
                -MathUtil.applyDeadband(two.getX(), Constants.JOYSTICK_DEADBAND));
    }

    private void configureBindings() {
        drive.setDefaultCommand(drive.defaultCommand(
                this::getChassisSpeeds,
                false));

        one.trigger().toggleOnTrue(drive.turtle());
        one.button(2).toggleOnTrue(drive.defaultCommand(
                this::getChassisSpeeds,
                true));

        two.trigger().onTrue(drive.zeroHeading());

        controller.leftTrigger().onTrue(intake.intake()).onFalse(intake.stop());
        controller.leftBumper().onTrue(intake.eject()).onFalse(intake.stop());
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}

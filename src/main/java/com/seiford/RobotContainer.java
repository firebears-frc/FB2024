package com.seiford;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.seiford.drive.Drive;
import com.seiford.subsystems.Intake;
import com.seiford.subsystems.Climber;
import com.seiford.subsystems.Arm;
import com.seiford.subsystems.Shooter;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    }

    private final Drive bass;
    private final Intake downbeat;
    private final Shooter upbeat;
    private final Climber glissando;
    private final Arm arm;

    private final CommandJoystick one;
    private final CommandJoystick two;
    private final CommandXboxController controller;
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        bass = new Drive();
        downbeat = new Intake();
        upbeat = new Shooter();
        glissando = new Climber();
        arm = new Arm();

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
        bass.setDefaultCommand(bass.defaultCommand(this::getChassisSpeeds, false));

        one.trigger().toggleOnTrue(bass.turtle());
        one.button(2).toggleOnTrue(bass.defaultCommand(this::getChassisSpeeds, true));

        two.trigger().onTrue(bass.zeroHeading());

        controller.leftTrigger().onTrue(downbeat.intake()).onFalse(downbeat.stop());
        controller.leftBumper().onTrue(downbeat.eject()).onFalse(downbeat.stop());
        controller.a().onTrue(upbeat.shoot()).onFalse(upbeat.stop());
        controller.x().onTrue(upbeat.eject()).onFalse(upbeat.stop());
        controller.povUp().onTrue(glissando.climb()).onFalse(glissando.stop());
        controller.povDown().onTrue(glissando.reverse()).onFalse(glissando.stop());
        arm.setDefaultCommand(arm.defaultCommand(controller::getLeftY));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}

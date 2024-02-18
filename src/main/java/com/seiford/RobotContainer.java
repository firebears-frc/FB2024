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
    // Constants
    private static final class Constants {
        public static final int JOYSTICK_1_PORT = 0;
        public static final int JOYSTICK_2_PORT = 1;
        public static final int CONTROLLER_PORT = 2;

        public static final double JOYSTICK_DEADBAND = 0.05;
        public static final double GAMEPAD_DEADBAND = 0.2;
    }

    // Objects
    private final Drive bass;
    private final Intake downbeat;
    private final Shooter upbeat;
    private final Climber glissando;
    private final Arm arm;

    private final CommandJoystick one;
    private final CommandJoystick two;
    private final CommandXboxController controller;
    private final LoggedDashboardChooser<Command> autoChooser;

    // Constructor
    public RobotContainer() {
        // Create subsystems
        bass = new Drive();
        downbeat = new Intake();
        upbeat = new Shooter();
        glissando = new Climber();
        arm = new Arm();

        // Create control interfaces
        one = new CommandJoystick(Constants.JOYSTICK_1_PORT);
        two = new CommandJoystick(Constants.JOYSTICK_2_PORT);
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);

        // Set up for autos
        NamedCommands.registerCommands(Map.of(
                "TODO", Commands.none()));
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());

        // Set up default commands
        bass.setDefaultCommand(bass.defaultCommand(this::getChassisSpeeds, false));
        arm.setDefaultCommand(
                arm.defaultCommand(() -> MathUtil.applyDeadband(controller.getLeftY(),
                        Constants.GAMEPAD_DEADBAND)));

        // Set up joystick one bindings
        one.trigger().toggleOnTrue(bass.turtle());
        one.button(2).toggleOnTrue(bass.defaultCommand(this::getChassisSpeeds, true));

        // Set up joystick two bindings
        two.trigger().onTrue(bass.zeroHeading());

        // Set up controller bindings
        controller.rightTrigger().onTrue(
                Commands.sequence(
                        arm.speaker(),
                        upbeat.speaker()))
                .onFalse(Commands.sequence(
                        downbeat.shoot(),
                        Commands.waitSeconds(0.5),
                        upbeat.stop(),
                        downbeat.stop(),
                        arm.stow()));
        controller.rightBumper().toggleOnTrue(Commands.parallel(arm.pickupStow(), downbeat.intakeStop()));
        controller.leftTrigger().onTrue(
                Commands.sequence(
                        arm.amp(),
                        upbeat.amp()))
                .onFalse(Commands.sequence(
                        downbeat.intake(),
                        Commands.waitSeconds(0.5),
                        upbeat.stop(),
                        downbeat.stop(),
                        arm.stow()));
        controller.leftBumper().onTrue(downbeat.eject()).onFalse(downbeat.stop());
        controller.povUp().onTrue(glissando.climb()).onFalse(glissando.stop());
        controller.povDown().onTrue(glissando.reverse()).onFalse(glissando.stop());
    }

    private ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(
                -MathUtil.applyDeadband(one.getY(), Constants.JOYSTICK_DEADBAND),
                -MathUtil.applyDeadband(one.getX(), Constants.JOYSTICK_DEADBAND),
                -MathUtil.applyDeadband(two.getX(), Constants.JOYSTICK_DEADBAND));
    }

    // Command factories
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}

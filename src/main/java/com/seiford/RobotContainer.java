package com.seiford;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.seiford.drive.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // Constants
    private static final class Constants {
        public static final int JOYSTICK_1_PORT = 0;
        public static final int JOYSTICK_2_PORT = 1;
        public static final int CONTROLLER_PORT = 2;

        public static final double JOYSTICK_DEADBAND = 0.05;
    }

    // Objects
    private final Drive bass;

    private final CommandJoystick one;
    private final CommandJoystick two;
    private final CommandXboxController controller;
    private final LoggedDashboardChooser<Command> autoChooser;

    // Constructor
    public RobotContainer() {
        // Create subsystems
        bass = new Drive();

        // Create control interfaces
        one = new CommandJoystick(Constants.JOYSTICK_1_PORT);
        two = new CommandJoystick(Constants.JOYSTICK_2_PORT);
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);

        // Set up for autos
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());

        // Set up default commands
        bass.setDefaultCommand(bass.defaultCommand(this::getChassisSpeeds, false));

        // Set up joystick one bindings
        one.trigger().toggleOnTrue(bass.turtle());
        one.button(2).toggleOnTrue(bass.defaultCommand(this::getChassisSpeeds, true));

        // Set up joystick two bindings
        two.trigger().onTrue(bass.zeroHeading());
    }

    private ChassisSpeeds getChassisSpeeds() {
        double x = MathUtil.applyDeadband(one.getY(), Constants.JOYSTICK_DEADBAND);
        double z = MathUtil.applyDeadband(one.getX(), Constants.JOYSTICK_DEADBAND);
        double omega = -MathUtil.applyDeadband(two.getX(), Constants.JOYSTICK_DEADBAND);

        if (Util.isRedAlliance()) {
            x *= -1.0;
            z *= -1.0;
        }

        return new ChassisSpeeds(x, z, omega);
    }

    // Command factories
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}

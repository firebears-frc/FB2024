package com.seiford;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.seiford.drive.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // Constants
    private static final class Constants {
        public static final int CONTROLLER_PORT = 0;

        public static final double CONTROLLER_DEADBAND = 0.1;
    }

    // Objects
    private final Drive bass;

    private final CommandXboxController controller;
    private final LoggedDashboardChooser<Command> autoChooser;

    // Constructor
    public RobotContainer() {
        // Create subsystems
        bass = new Drive();

        // Create control interfaces
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);

        // Set up for autos
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());

        // Set up default commands
        bass.setDefaultCommand(bass.defaultCommand(this::getChassisSpeeds, false));

        // Set up controller bindings
        controller.rightStick().toggleOnTrue(bass.turtle());
        controller.leftStick().toggleOnTrue(bass.defaultCommand(this::getChassisSpeeds, true));
        controller.start().onTrue(bass.zeroHeading());
    }

    private ChassisSpeeds getChassisSpeeds() {
        double x = MathUtil.applyDeadband(controller.getLeftY(), Constants.CONTROLLER_DEADBAND);
        double z = MathUtil.applyDeadband(controller.getLeftX(), Constants.CONTROLLER_DEADBAND);
        double omega = -MathUtil.applyDeadband(controller.getRightX(), Constants.CONTROLLER_DEADBAND);

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

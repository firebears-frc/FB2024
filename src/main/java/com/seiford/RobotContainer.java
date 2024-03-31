package com.seiford;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.seiford.drive.Drive;
import com.seiford.subsystems.Intake;
import com.seiford.subsystems.Climber;
import com.seiford.subsystems.Arm;
import com.seiford.subsystems.Shooter;
import com.seiford.subsystems.Conductor;

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

        public static final double SHOOT_DELAY = 0.25;
    }

    // Objects
    private final Drive bass;
    private final Intake downbeat;
    private final Conductor conductor;
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
        conductor = new Conductor(bass::getPose);
        upbeat = new Shooter(conductor::getShooterRPM);
        glissando = new Climber(bass::getRoll);
        arm = new Arm(conductor::getArmAngle);

        // Create control interfaces
        one = new CommandJoystick(Constants.JOYSTICK_1_PORT);
        two = new CommandJoystick(Constants.JOYSTICK_2_PORT);
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);

        // Set up for autos
        NamedCommands.registerCommands(Map.of(
                "GroundSlam", arm.groundSlam(),
                "Shoot", Commands.sequence(
                        downbeat.shoot(),
                        Commands.waitSeconds(Constants.SHOOT_DELAY),
                        Commands.parallel(
                                upbeat.stop(),
                                downbeat.stop())),
                "Intake", Commands.parallel(
                        arm.pickup(),
                        downbeat.autoIntake()),
                "PrepareShoot", Commands.parallel(
                        arm.speaker(),
                        upbeat.speaker())));
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());

        // Set up default commands
        bass.setDefaultCommand(bass.defaultCommand(this::getChassisSpeeds, false));

        // Set up joystick one bindings
        one.trigger().toggleOnTrue(bass.turtle());
        one.button(2).toggleOnTrue(bass.defaultCommand(this::getChassisSpeeds, true));

        // Set up joystick two bindings
        two.trigger().onTrue(bass.zeroHeading());

        // Set up controller bindings
        controller.rightTrigger().onTrue(
                Commands.parallel(
                        arm.speaker(),
                        upbeat.speaker()))
                .onFalse(Commands.sequence(
                        downbeat.shoot(),
                        Commands.waitSeconds(Constants.SHOOT_DELAY),
                        Commands.parallel(
                                upbeat.stop(),
                                downbeat.stop())));
        controller.rightBumper().onTrue(Commands.sequence(
                arm.pickup(),
                downbeat.autoIntake(),
                arm.speaker()));
        controller.leftTrigger().onTrue(
                Commands.parallel(
                        arm.amp(),
                        upbeat.amp()))
                .onFalse(Commands.sequence(
                        downbeat.intake(),
                        Commands.waitSeconds(Constants.SHOOT_DELAY),
                        Commands.parallel(
                                upbeat.stop(),
                                downbeat.stop(),
                                arm.speaker())));
        controller.leftBumper().onTrue(downbeat.eject()).onFalse(downbeat.stop());
        controller.povUp().onTrue(glissando.climb()).onFalse(glissando.stop());
        controller.povDown().onTrue(glissando.reverse()).onFalse(glissando.stop());
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

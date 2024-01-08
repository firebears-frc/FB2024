package frc.robot.drive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DefaultCommand extends Command {
    private static final class Constants {
        public static final double SLOW_TELE_VELOCITY = 1.0; // meters per second
        public static final double MAX_TELE_ANGULAR_VELOCITY = 1.5 * Math.PI; // radians per second
        public static final double SLOW_TELE_ANGULAR_VELOCITY = Math.PI / 2; // radians per second
    }

    private final Supplier<ChassisSpeeds> commandSupplier;
    private final Consumer<ChassisSpeeds> commandConsumer;
    private final boolean slowMode;
    private final RateLimiter rateLimiter = new RateLimiter();

    public DefaultCommand(Supplier<ChassisSpeeds> commandSupplier, Consumer<ChassisSpeeds> commandConsumer,
            boolean slowMode, Subsystem requirement) {
        this.commandSupplier = commandSupplier;
        this.commandConsumer = commandConsumer;
        this.slowMode = slowMode;

        addRequirements(requirement);
    }

    @Override
    public void execute() {
        ChassisSpeeds command = commandSupplier.get();
        command = rateLimiter.calculate(command);

        if (slowMode) {
            command.vxMetersPerSecond *= Constants.SLOW_TELE_VELOCITY;
            command.vyMetersPerSecond *= Constants.SLOW_TELE_VELOCITY;
            command.omegaRadiansPerSecond *= Constants.SLOW_TELE_ANGULAR_VELOCITY;
        } else {
            command.vxMetersPerSecond *= Chassis.MAX_VELOCITY;
            command.vyMetersPerSecond *= Chassis.MAX_VELOCITY;
            command.omegaRadiansPerSecond *= Constants.MAX_TELE_ANGULAR_VELOCITY;
        }

        commandConsumer.accept(command);
    }
}

package frc.robot.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.WPIUtilJNI;

public class RateLimiter {
    private static final class Constants {
        // Slew Rate Limiting
        public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
        public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
        public static final double ROTATION_SLEW_RATE = 2.0; // percent per second (1 = 100%)
    }

    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(Constants.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.ROTATION_SLEW_RATE);
    @AutoLogOutput(key = "Drive/RateLimiter/CurrentDirection")
    private double currentDirection = 0.0;
    @AutoLogOutput(key = "Drive/RateLimiter/CurrentMagnitude")
    private double currentMagnitude = 0.0;
    private double previousTime = WPIUtilJNI.now() * 1e-6;

    private static double angleDifference(double angleOne, double angleTwo) {
        double difference = Math.abs(angleOne - angleTwo);
        return difference > Math.PI ? (2 * Math.PI) - difference : difference;
    }

    private static double wrapAngle(double angle) {
        if (angle == (Math.PI * 2)) {
            return 0.0;
        } else if (angle > (Math.PI * 2)) {
            return angle - (Math.PI * 2) * Math.floor(angle / (Math.PI * 2));
        } else if (angle < 0.0) {
            return angle + (Math.PI * 2) * Math.floor((-angle / (Math.PI * 2)) + 1);
        } else {
            return angle;
        }
    }

    private static double stepTowardsCircular(double current, double target, double step) {
        current = wrapAngle(current);
        target = wrapAngle(target);

        double direction = Math.signum(target - current);
        double difference = Math.abs(current - target);

        if (difference <= step) {
            return target;
        }

        if (difference > Math.PI) {
            if (current + (2 * Math.PI) - target < step || target + (2 * Math.PI) - current < step) {
                return target;
            } else {
                return wrapAngle(current - direction * step);
            }
        }

        return current + direction * step;
    }

    public ChassisSpeeds calculate(ChassisSpeeds command) {
        // Convert XY to polar for rate limiting
        Logger.recordOutput("Drive/RateLimiter/Input", command);
        double inputDirection = Math.atan2(command.vyMetersPerSecond, command.vxMetersPerSecond);
        double inputMagnitude = Math
                .sqrt(Math.pow(command.vxMetersPerSecond, 2) + Math.pow(command.vyMetersPerSecond, 2));

        // Calculate the direction slew rate based on an estimate of the lateral
        // acceleration
        double directionSlewRate;
        if (currentMagnitude != 0.0) {
            directionSlewRate = Math.abs(Constants.DIRECTION_SLEW_RATE / currentMagnitude);
        } else {
            directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
        }

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - previousTime;
        double angleDif = angleDifference(inputDirection, currentDirection);
        if (angleDif < 0.45 * Math.PI) {
            currentDirection = stepTowardsCircular(currentDirection, inputDirection,
                    directionSlewRate * elapsedTime);
            currentMagnitude = magnitudeLimiter.calculate(inputMagnitude);
        } else if (angleDif > 0.85 * Math.PI) {
            if (currentMagnitude > 0.01) {
                currentMagnitude = magnitudeLimiter.calculate(0.0);
            } else {
                currentDirection = wrapAngle(currentDirection + Math.PI);
                currentMagnitude = magnitudeLimiter.calculate(inputMagnitude);
            }
        } else {
            currentDirection = stepTowardsCircular(currentDirection, inputDirection,
                    directionSlewRate * elapsedTime);
            currentMagnitude = magnitudeLimiter.calculate(0.0);
        }
        previousTime = currentTime;

        command.vxMetersPerSecond = currentMagnitude * Math.cos(currentDirection);
        command.vyMetersPerSecond = currentMagnitude * Math.sin(currentDirection);
        command.omegaRadiansPerSecond = rotationLimiter.calculate(command.omegaRadiansPerSecond);

        Logger.recordOutput("Drive/RateLimiter/Output", command);
        return command;
    }
}

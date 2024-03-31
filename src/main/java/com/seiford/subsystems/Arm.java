package com.seiford.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.seiford.util.spark.ClosedLoopConfiguration;
import com.seiford.util.spark.CurrentLimitConfiguration;
import com.seiford.util.spark.FeedbackConfiguration;
import com.seiford.util.spark.FollowingConfiguration;
import com.seiford.util.spark.SparkConfiguration;
import com.seiford.util.spark.StatusFrameConfiguration;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    // Constants
    private static final class Constants {
        public static final int RIGHT_CAN_ID = 13;
        public static final int LEFT_CAN_ID = 12;

        public static final SparkConfiguration RIGHT_CONFIG = new SparkConfiguration(
                true,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(20, 20, 10, 30.0),
                StatusFrameConfiguration.absoluteEncoderLeader(),
                ClosedLoopConfiguration.wrapping(0.02, 0.0, 0.0, 0.0, 0, 360),
                FeedbackConfiguration.absoluteEncoder(true, 360));
        public static final SparkConfiguration LEFT_CONFIG = new SparkConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(20, 20, 10, 30.0),
                StatusFrameConfiguration.normal(),
                FollowingConfiguration.spark(RIGHT_CAN_ID, true));

        public static final Rotation2d MANUAL_SPEED = Rotation2d.fromDegrees(1.0); // per loop

        public static final Rotation2d MIN = Rotation2d.fromDegrees(-5.0);
        public static final Rotation2d PICKUP = Rotation2d.fromDegrees(0.0);
        public static final Rotation2d SPEAKER = Rotation2d.fromDegrees(14.5);
        public static final Rotation2d STOW = Rotation2d.fromDegrees(20.0);
        public static final Rotation2d AMP = Rotation2d.fromDegrees(90.0);
        public static final Rotation2d MAX = Rotation2d.fromDegrees(100);

        public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(1.0);

        public static final double kG = 0.35; // volts to hold horizontal

        public static final double DEBOUNCE_TIME = 0.05;
    }

    // Objects
    private final CANSparkMax rightMotor, leftMotor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkPIDController pid;
    private final Debouncer debouncer;

    private Rotation2d setpoint;

    // Constructor
    public Arm() {
        rightMotor = new CANSparkMax(Constants.RIGHT_CAN_ID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.LEFT_CAN_ID, MotorType.kBrushless);
        encoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pid = rightMotor.getPIDController();
        debouncer = new Debouncer(Constants.DEBOUNCE_TIME);

        Constants.RIGHT_CONFIG.apply(rightMotor);
        Constants.LEFT_CONFIG.apply(leftMotor);

        setpoint = getAngle();
    }

    // Interface functions
    @AutoLogOutput(key = "Arm/Target")
    private Rotation2d getTargetAngle() {
        return setpoint;
    }

    @AutoLogOutput(key = "Arm/Actual")
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

    @AutoLogOutput(key = "Arm/Error")
    private Rotation2d getError() {
        return getAngle().minus(getTargetAngle());
    }

    @AutoLogOutput(key = "Arm/NearTarget")
    private boolean nearTarget() {
        return Math.abs(getError().getDegrees()) < Constants.TOLERANCE.getDegrees();
    }

    @AutoLogOutput(key = "Arm/OnTarget")
    private boolean onTarget() {
        return debouncer.calculate(nearTarget());
    }

    @Override
    public void periodic() {
        double feedforward = Math.cos(getAngle().getDegrees()) * Constants.kG;
        pid.setReference(setpoint.getDegrees(), ControlType.kPosition, 0, feedforward);

        Logger.recordOutput("Arm/Left/Output", leftMotor.getAppliedOutput());
        Logger.recordOutput("Arm/Right/Output", rightMotor.getAppliedOutput());
        Logger.recordOutput("Arm/Feedforward", feedforward);
    }

    private void set(Rotation2d angle) {
        if (angle.getDegrees() > Constants.MAX.getDegrees()) {
            setpoint = Constants.MAX;
        } else if (angle.getDegrees() < Constants.MIN.getDegrees()) {
            setpoint = Constants.MIN;
        } else {
            setpoint = angle;
        }
    }

    // Command factories
    private Command positionCommand(Rotation2d angle) {
        return Commands.sequence(
                runOnce(() -> set(angle)),
                Commands.waitSeconds(Constants.DEBOUNCE_TIME),
                run(() -> {}).until(this::onTarget)
        );
    }

    public Command pickupStow() {
        return startEnd(() -> set(Constants.PICKUP), () -> set(Constants.STOW));
    }

    public Command pickup() {
        return positionCommand(Constants.PICKUP);
    }

    public Command stow() {
        return positionCommand(Constants.STOW);
    }

    public Command amp() {
        return positionCommand(Constants.AMP);
    }

    public Command speaker() {
        return positionCommand(Constants.SPEAKER);
    }

    public Command defaultCommand(Supplier<Double> change) {
        return run(() -> set(setpoint.plus(Constants.MANUAL_SPEED.times(change.get()))));
    }
}

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

        public static final Rotation2d PICKUP_ANGLE = Rotation2d.fromDegrees(0.0);
        public static final Rotation2d AMP_ANGLE = Rotation2d.fromDegrees(90.0);

        public static final Rotation2d PICKUP_TOLERANCE = Rotation2d.fromDegrees(2.5);
        public static final Rotation2d SPEAKER_TOLERANCE = Rotation2d.fromDegrees(0.75);
        public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(1.0);

        public static final double kG = 0.35; // volts to hold horizontal

        public static final double DEBOUNCE_TIME = 0.05;
    }

    private static enum State {
        STARTUP,
        PICKUP,
        SPEAKER,
        AMP
    };

    // Objects
    private final CANSparkMax rightMotor, leftMotor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkPIDController pid;
    private final Debouncer debouncer;
    private final Supplier<Rotation2d> angleSupplier;

    @AutoLogOutput(key = "Arm/State")
    private State state = State.STARTUP;

    // Constructor
    public Arm(Supplier<Rotation2d> angleSupplier) {
        rightMotor = new CANSparkMax(Constants.RIGHT_CAN_ID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.LEFT_CAN_ID, MotorType.kBrushless);
        encoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pid = rightMotor.getPIDController();
        debouncer = new Debouncer(Constants.DEBOUNCE_TIME);
        this.angleSupplier = angleSupplier;

        Constants.RIGHT_CONFIG.apply(rightMotor);
        Constants.LEFT_CONFIG.apply(leftMotor);
    }

    // Interface functions
    @AutoLogOutput(key = "Arm/Target")
    private Rotation2d getTargetAngle() {
        return switch (state) {
            case STARTUP -> getAngle();
            case PICKUP -> Constants.PICKUP_ANGLE;
            case SPEAKER -> angleSupplier.get();
            case AMP -> Constants.AMP_ANGLE;
        };
    }

    @AutoLogOutput(key = "Arm/Actual")
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

    @AutoLogOutput(key = "Arm/Error")
    private Rotation2d getError() {
        return getAngle().minus(getTargetAngle());
    }

    @AutoLogOutput(key = "Arm/Tolerance")
    private Rotation2d getTolerance() {
        return switch (state) {
            case PICKUP -> Constants.PICKUP_TOLERANCE;
            case SPEAKER -> Constants.SPEAKER_TOLERANCE;
            default -> Constants.TOLERANCE;
        };
    }

    @AutoLogOutput(key = "Arm/NearTarget")
    private boolean nearTarget() {
        return Math.abs(getError().getDegrees()) < getTolerance().getDegrees();
    }

    @AutoLogOutput(key = "Arm/OnTarget")
    private boolean onTarget() {
        return debouncer.calculate(nearTarget());
    }

    @Override
    public void periodic() {
        double feedforward = Math.cos(getAngle().getDegrees()) * Constants.kG;
        pid.setReference(getTargetAngle().getDegrees(), ControlType.kPosition, 0, feedforward);

        Logger.recordOutput("Arm/Left/Output", leftMotor.getAppliedOutput());
        Logger.recordOutput("Arm/Right/Output", rightMotor.getAppliedOutput());
        Logger.recordOutput("Arm/Feedforward", feedforward);
    }

    // Command factories
    private Command stateCommand(State target) {
        return Commands.sequence(
                runOnce(() -> state = target),
                Commands.waitSeconds(Constants.DEBOUNCE_TIME),
                run(() -> {}).until(this::onTarget));
    }

    public Command pickup() {
        return stateCommand(State.PICKUP);
    }

    public Command amp() {
        return stateCommand(State.AMP);
    }

    public Command speaker() {
        return stateCommand(State.SPEAKER);
    }

    public Command groundSlam() {
        return Commands.sequence(
                runOnce(() -> state = State.PICKUP),
                Commands.waitSeconds(Constants.DEBOUNCE_TIME),
                run(() -> {}).until(() -> getAngle().getDegrees() < 10.0));
    }
}

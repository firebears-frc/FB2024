package com.seiford.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    // Constants
    private static final class Constants {
        public static final int RIGHT_CAN_ID = 12;
        public static final int LEFT_CAN_ID = 13;

        public static final SparkConfiguration RIGHT_CONFIG = new SparkConfiguration(
                true,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(30, 20, 10, 35.0),
                StatusFrameConfiguration.absoluteEncoderLeader(),
                ClosedLoopConfiguration.wrapping(0.0175, 0.0, 0.005, 0.0, 0, 360),
                FeedbackConfiguration.absoluteEncoder(true, 360));
        public static final SparkConfiguration LEFT_CONFIG = new SparkConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(30, 20, 10, 35.0),
                StatusFrameConfiguration.normal(),
                FollowingConfiguration.spark(RIGHT_CAN_ID, true));

        public static final Rotation2d MANUAL_SPEED = Rotation2d.fromDegrees(1.0); // per loop

        public static final Rotation2d PICKUP = Rotation2d.fromDegrees(-3.0);
        public static final Rotation2d STOW = Rotation2d.fromDegrees(20.0);
        public static final Rotation2d AMP = Rotation2d.fromDegrees(85.0);
        public static final Rotation2d SPEAKER = Rotation2d.fromDegrees(35.0);

        public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.0);
    }

    // Objects
    private final CANSparkMax rightMotor, leftMotor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkPIDController pid;

    private Rotation2d setpoint;

    // Constructor
    public Arm() {
        rightMotor = new CANSparkMax(Constants.RIGHT_CAN_ID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.LEFT_CAN_ID, MotorType.kBrushless);
        encoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pid = rightMotor.getPIDController();

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

    @AutoLogOutput(key = "Arm/OnTarget")
    private boolean onTarget() {
        return Math.abs(getError().getDegrees()) < Constants.TOLERANCE.getDegrees();
    }

    @Override
    public void periodic() {
        pid.setReference(setpoint.getDegrees(), ControlType.kPosition);
    }

    // Command factories
    private Command positionCommand(Rotation2d angle) {
        return run(() -> setpoint = angle).until(this::onTarget);
    }

    public Command pickupStow() {
        return startEnd(() -> setpoint = Constants.PICKUP, () -> setpoint = Constants.STOW);
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
        return run(() -> setpoint = setpoint.plus(Constants.MANUAL_SPEED.times(change.get())));
    }
}

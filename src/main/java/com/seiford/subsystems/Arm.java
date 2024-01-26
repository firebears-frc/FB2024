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
    private static final class Constants {
        public static final int RIGHT_CAN_ID = 12;
        public static final int LEFT_CAN_ID = 13;

        public static final SparkConfiguration CONFIG_RIGHT = new SparkConfiguration(
                true,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(30, 20, 10, 35.0),
                StatusFrameConfiguration.absoluteEncoderLeader(),
                ClosedLoopConfiguration.wrapping(0.0175, 0.0, 0.005, 0.0, 0, 360),
                FeedbackConfiguration.absoluteEncoder(true, 360));
        public static final SparkConfiguration CONFIG_LEFT = new SparkConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(30, 20, 10, 35.0),
                StatusFrameConfiguration.normal(),
                FollowingConfiguration.spark(RIGHT_CAN_ID, true));

        public static final Rotation2d MANUAL_SPEED = Rotation2d.fromDegrees(1.0); // per loop
    }

    private final CANSparkMax motorRight, motorLeft;
    private final SparkAbsoluteEncoder encoder;
    private final SparkPIDController pid;

    protected Rotation2d setpoint = new Rotation2d();
    protected Rotation2d position = new Rotation2d();

    public Arm() {
        motorRight = new CANSparkMax(Constants.RIGHT_CAN_ID, MotorType.kBrushless);
        motorLeft = new CANSparkMax(Constants.LEFT_CAN_ID, MotorType.kBrushless);
        encoder = motorRight.getAbsoluteEncoder(Type.kDutyCycle);
        pid = motorRight.getPIDController();

        Constants.CONFIG_RIGHT.apply(motorRight);
        Constants.CONFIG_LEFT.apply(motorLeft);
    }

    public void setAngle(Rotation2d angle) {
        setpoint = angle;
    }

    @AutoLogOutput(key = "Shoulder/Setpoint")
    public Rotation2d getTargetAngle() {
        return setpoint;
    }

    @AutoLogOutput(key = "Shoulder/Position")
    public Rotation2d getAngle() {
        return position;
    }

    @AutoLogOutput(key = "Shoulder/Error")
    public Rotation2d getError() {
        return getAngle().minus(getTargetAngle());
    }

    @Override
    public void periodic() {
        position = Rotation2d.fromDegrees(encoder.getPosition());

        pid.setReference(setpoint.getDegrees(), ControlType.kPosition);
    }

    public Command defaultCommand(Supplier<Double> change) {
        return run(() -> {
            setpoint = setpoint.plus(Constants.MANUAL_SPEED.times(change.get()));

            setAngle(setpoint);
        });
    }
}

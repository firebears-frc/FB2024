package com.seiford.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.seiford.util.spark.ClosedLoopConfiguration;
import com.seiford.util.spark.CurrentLimitConfiguration;
import com.seiford.util.spark.FeedbackConfiguration;
import com.seiford.util.spark.FollowingConfiguration;
import com.seiford.util.spark.SparkConfiguration;
import com.seiford.util.spark.StatusFrameConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Glissando extends SubsystemBase {
    private static final class Constants {
        public static final int RIGHT_MOTOR_CAN_ID = 10;
        public static final int LEFT_MOTOR_CAN_ID = 100;

        public static final SparkConfiguration CONFIG = new SparkConfiguration(
                false,
                IdleMode.kCoast,
                CurrentLimitConfiguration.complex(40, 20, 10, 45.0),
                StatusFrameConfiguration.normal(),
                ClosedLoopConfiguration.simple(1.0, 0.0, 0.0, 0.0), // TODO!
                FeedbackConfiguration.builtInEncoder(false, 1));

        public static final double CLIMB_SPEED = 0.25;
        public static final double REVERSE_SPEED = -0.25;
    }

    private enum State {
        CLIMB,
        REVERSE,
        STOP
    }

    private final CANSparkMax left_motor, right_motor;

    @AutoLogOutput(key = "Glissando/State")
    private State state = State.STOP;
    @AutoLogOutput(key = "Glissando/Speed")
    private double speed;

    public Glissando() {
        left_motor = new CANSparkMax(Constants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        right_motor = new CANSparkMax(Constants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

        Constants.CONFIG.apply(left_motor, right_motor);

        left_motor.burnFlash();
        right_motor.burnFlash();
    }

    public Command climb() {
        return runOnce(() -> state = State.CLIMB);
    }

    public Command reverse() {
        return runOnce(() -> state = State.REVERSE);
    }

    public Command stop() {
        return runOnce(() -> state = State.STOP);
    }

    @Override
    public void periodic() {
        // Figure out what speed we should be running
        speed = switch (state) {
            case CLIMB -> Constants.CLIMB_SPEED;
            case REVERSE -> Constants.REVERSE_SPEED;
            case STOP -> 0;
        };

        // Update the position controller
        left_motor.set(speed);
        right_motor.set(speed);
    }
}

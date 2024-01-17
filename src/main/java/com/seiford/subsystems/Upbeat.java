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

public class Upbeat extends SubsystemBase {
    private static final class Constants {
        public static final int TOP_MOTOR_CAN_ID = 10;
        public static final int BOTTOM_MOTOR_CAN_ID = 100;

        public static final SparkConfiguration TOP_CONFIG = new SparkConfiguration(
                false,
                IdleMode.kCoast,
                CurrentLimitConfiguration.complex(40, 20, 10, 45.0),
                StatusFrameConfiguration.leader(),
                ClosedLoopConfiguration.simple(3.0, 0.0, 0.0, 0.0), // TODO!
                FeedbackConfiguration.builtInEncoder(false, 1));
        public static final SparkConfiguration BOTTOM_CONFIG = new SparkConfiguration(
                false,
                IdleMode.kCoast,
                CurrentLimitConfiguration.complex(40, 20, 10, 45.0),
                StatusFrameConfiguration.normal(),
                FollowingConfiguration.spark(TOP_MOTOR_CAN_ID, false));

        public static final double SHOOT_SPEED = 5000 * 60; // rotations per second
        public static final double EJECT_SPEED = -2500 * 60; // rotations per second
    }

    private enum State {
        SHOOT,
        EJECT,
        STOP
    }

    private final CANSparkMax top_motor, bottom_motor;
    private final RelativeEncoder top_encoder, bottom_encoder;
    private final SparkPIDController pid;

    @AutoLogOutput(key = "Upbeat/State")
    private State state = State.STOP;
    @AutoLogOutput(key = "Upbeat/Speed")
    private double speed;

    public Upbeat() {
        top_motor = new CANSparkMax(Constants.TOP_MOTOR_CAN_ID, MotorType.kBrushless);
        bottom_motor = new CANSparkMax(Constants.BOTTOM_MOTOR_CAN_ID, MotorType.kBrushless);
        top_encoder = top_motor.getEncoder();
        bottom_encoder = bottom_motor.getEncoder();
        pid = top_motor.getPIDController();

        Constants.TOP_CONFIG.apply(top_motor);
        Constants.BOTTOM_CONFIG.apply(bottom_motor);

        top_motor.burnFlash();
        bottom_motor.burnFlash();
    }

    public Command shoot() {
        return runOnce(() -> state = State.SHOOT);
    }

    public Command eject() {
        return runOnce(() -> state = State.EJECT);
    }

    public Command stop() {
        return runOnce(() -> state = State.STOP);
    }

    @Override
    public void periodic() {
        // Figure out what speed we should be running
        speed = switch (state) {
            case SHOOT -> Constants.SHOOT_SPEED;
            case EJECT -> Constants.EJECT_SPEED;
            case STOP -> 0;
        };

        // Log actual velocities
        Logger.recordOutput("Upbeat/TopSpeed", top_encoder.getVelocity());
        Logger.recordOutput("Upbeat/BottomSpeed", bottom_encoder.getVelocity());

        // Update the position controller
        pid.setReference(speed, ControlType.kVelocity);
    }
}

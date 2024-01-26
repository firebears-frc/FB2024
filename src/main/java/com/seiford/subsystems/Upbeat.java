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
        public static final int BOTTOM_MOTOR_CAN_ID = 11;

        public static final SparkConfiguration TOP_CONFIG = new SparkConfiguration(
                false,
                IdleMode.kCoast,
                CurrentLimitConfiguration.complex(40, 20, 10, 45.0),
                StatusFrameConfiguration.leader(),
                ClosedLoopConfiguration.simple(3.0, 0.0, 0.0, 0.0), // TODO!
                FeedbackConfiguration.builtInEncoder(1));
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

    private final CANSparkMax motorTop, motorBottom;
    private final RelativeEncoder encoderTop, encoderBottom;
    private final SparkPIDController pid;

    @AutoLogOutput(key = "Upbeat/State")
    private State state = State.STOP;
    @AutoLogOutput(key = "Upbeat/Setpoint")
    private double speed;

    public Upbeat() {
        motorTop = new CANSparkMax(Constants.TOP_MOTOR_CAN_ID, MotorType.kBrushless);
        motorBottom = new CANSparkMax(Constants.BOTTOM_MOTOR_CAN_ID, MotorType.kBrushless);
        encoderTop = motorTop.getEncoder();
        encoderBottom = motorBottom.getEncoder();
        pid = motorTop.getPIDController();

        Constants.TOP_CONFIG.apply(motorTop);
        Constants.BOTTOM_CONFIG.apply(motorBottom);

        motorTop.burnFlash();
        motorBottom.burnFlash();
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
        Logger.recordOutput("Upbeat/TopSpeed", encoderTop.getVelocity());
        Logger.recordOutput("Upbeat/BottomSpeed", encoderBottom.getVelocity());

        // Update the position controller
        pid.setReference(speed, ControlType.kVelocity);
    }
}

package com.seiford.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.seiford.util.spark.ClosedLoopConfiguration;
import com.seiford.util.spark.CurrentLimitConfiguration;
import com.seiford.util.spark.FeedbackConfiguration;
import com.seiford.util.spark.SparkConfiguration;
import com.seiford.util.spark.StatusFrameConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Downbeat extends SubsystemBase {
    private static final class Constants {
        public static final int MOTOR_CAN_ID = 9;

        public static final SparkConfiguration CONFIG = new SparkConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(20, 10, 10, 25.0),
                StatusFrameConfiguration.normal(),
                ClosedLoopConfiguration.simple(3.0, 0.0, 0.0, 0.0),
                FeedbackConfiguration.builtInEncoder(1));

        public static final double INTAKE_SPEED = 0.01; // rotations per cycle
        public static final double EJECT_SPEED = 0.01; // rotations per cycle
    }

    private enum State {
        INTAKE,
        EJECT,
        STOP
    }

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;

    @AutoLogOutput(key = "Downbeat/State")
    private State state = State.STOP;
    @AutoLogOutput(key = "Downbeat/Speed")
    private double speed;
    @AutoLogOutput(key = "Downbeat/Actual")
    private double actual;
    @AutoLogOutput(key = "Downbeat/Target")
    private double target;

    public Downbeat() {
        motor = new CANSparkMax(Constants.MOTOR_CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();

        Constants.CONFIG.apply(motor);

        motor.burnFlash();
    }

    public Command intake() {
        return runOnce(() -> state = State.INTAKE);
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
            case INTAKE -> Constants.INTAKE_SPEED;
            case EJECT -> Constants.EJECT_SPEED;
            case STOP -> 0;
        };

        // Update the position controller
        actual = encoder.getPosition();
        target = speed + actual;
        pid.setReference(target, ControlType.kPosition);
    }
}

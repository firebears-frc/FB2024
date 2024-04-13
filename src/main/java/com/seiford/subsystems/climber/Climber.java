package com.seiford.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.seiford.util.spark.ClosedLoopConfiguration;
import com.seiford.util.spark.CurrentLimitConfiguration;
import com.seiford.util.spark.FeedbackConfiguration;
import com.seiford.util.spark.SparkConfiguration;
import com.seiford.util.spark.StatusFrameConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    // Constants
    private static final class Constants {
        public static final int RIGHT_CAN_ID = 14;
        public static final int LEFT_CAN_ID = 15;

        public static final SparkConfiguration CONFIG = new SparkConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(40, 20, 10, 45.0),
                StatusFrameConfiguration.normal(),
                ClosedLoopConfiguration.simple(1.0, 0.0, 0.0, 0.0),
                FeedbackConfiguration.builtInEncoder(1));

        public static final double FULL_CLIMB_SPEED = 1.0;
        public static final double FULL_REVERSE_SPEED = -1.0;
        public static final double SLOW_CLIMB_SPEED = 0.5;
        public static final double SLOW_REVERSE_SPEED = -0.5;
        public static final double LEVELING_SPEED = 0.25;

        public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.5);
    }

    private static enum State {
        INACTIVE,
        FULL_CLIMB,
        SLOW_CLIMB,
        FULL_REVERSE,
        SLOW_REVERSE,
        BALANCING
    };

    // Objects
    private final CANSparkMax rightMotor, leftMotor;
    private final Supplier<Rotation2d> rollSupplier;

    @AutoLogOutput(key = "Climber/State")
    private State state = State.INACTIVE;

    // Constructor
    public Climber(Supplier<Rotation2d> rollSupplier) {
        rightMotor = new CANSparkMax(Constants.RIGHT_CAN_ID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.LEFT_CAN_ID, MotorType.kBrushless);

        Constants.CONFIG.apply(rightMotor);
        Constants.CONFIG.apply(leftMotor);

        rightMotor.burnFlash();
        leftMotor.burnFlash();

        this.rollSupplier = rollSupplier;
    }

    @AutoLogOutput(key = "Climber/IsLevel")
    private boolean isLevel() {
        return Math.abs(rollSupplier.get().getDegrees()) < Constants.TOLERANCE.getDegrees();
    }

    private void set(double left, double right) {
        rightMotor.set(right);
        leftMotor.set(left);
    }

    private void balance(double speed) {
        if (isLevel()) {
            set(speed, speed);
        } else if (rollSupplier.get().getDegrees() > 0.0) {
            set(speed - Constants.LEVELING_SPEED, speed + Constants.LEVELING_SPEED);
        } else {
            set(speed + Constants.LEVELING_SPEED, speed - Constants.LEVELING_SPEED);
        }
    }

    @Override
    public void periodic() {
        switch (state) {
            case INACTIVE -> set(0.0, 0.0);
            case FULL_CLIMB -> set(Constants.FULL_CLIMB_SPEED, Constants.FULL_CLIMB_SPEED);
            case SLOW_CLIMB -> balance(Constants.SLOW_CLIMB_SPEED);
            case FULL_REVERSE -> set(Constants.FULL_REVERSE_SPEED, Constants.FULL_REVERSE_SPEED);
            case SLOW_REVERSE -> balance(Constants.SLOW_REVERSE_SPEED);
            case BALANCING -> balance(0.0);
        }
    }

    private Command stateCommand(State target) {
        return runOnce(() -> state = target);
    }

    // Command factories
    public Command climb() {
        return stateCommand(State.FULL_CLIMB);
    }

    public Command reverse() {
        return stateCommand(State.FULL_REVERSE);
    }

    public Command stop() {
        return stateCommand(State.BALANCING);
    }
}

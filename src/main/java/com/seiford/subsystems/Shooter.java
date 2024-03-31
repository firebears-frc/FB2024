package com.seiford.subsystems;

import java.util.function.Supplier;

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
import com.seiford.util.spark.SparkConfiguration;
import com.seiford.util.spark.StatusFrameConfiguration;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // Constants
    private static final class Constants {
        public static final int TOP_CAN_ID = 10;
        public static final int BOTTOM_CAN_ID = 11;

        public static final SparkConfiguration CONFIG = new SparkConfiguration(
                false,
                IdleMode.kCoast,
                CurrentLimitConfiguration.complex(50, 30, 10, 60.0),
                StatusFrameConfiguration.normal(),
                ClosedLoopConfiguration.outputConstraints(0.0003, 0.0000001, 0.0, 0.0001875, 0.0, 1.0),
                FeedbackConfiguration.builtInEncoder(1));

        public static final double AMP_SPEED = 1000; // rotations per minute
        public static final double EJECT_SPEED = -1000; // rotations per minute

        public static final double TOLERANCE = 100; // rotations per minute

        public static final double DEBOUNCE_TIME = 0.05;
    }

    private static enum State {
        STOPPED,
        SPEAKER,
        AMP,
        EJECT
    };

    // Objects
    private final CANSparkMax topMotor, bottomMotor;
    private final RelativeEncoder topEncoder, bottomEncoder;
    private final SparkPIDController topPID, bottomPID;
    private final Debouncer debouncer;
    private final Supplier<Double> speedSupplier;

    @AutoLogOutput(key = "Shooter/State")
    private State state = State.STOPPED;

    // Constructor
    public Shooter(Supplier<Double> speedSupplier) {
        topMotor = new CANSparkMax(Constants.TOP_CAN_ID, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(Constants.BOTTOM_CAN_ID, MotorType.kBrushless);
        topEncoder = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();
        topPID = topMotor.getPIDController();
        bottomPID = bottomMotor.getPIDController();
        debouncer = new Debouncer(Constants.DEBOUNCE_TIME);
        this.speedSupplier = speedSupplier;

        Constants.CONFIG.apply(topMotor, bottomMotor);

        topMotor.burnFlash();
        bottomMotor.burnFlash();
    }

    // Interface functions
    @AutoLogOutput(key = "Shooter/Setpoint")
    private double getTargetVelocity() {
        return switch (state) {
            case AMP -> Constants.AMP_SPEED;
            case EJECT -> Constants.EJECT_SPEED;
            case SPEAKER -> speedSupplier.get();
            case STOPPED -> 0.0;
        };
    }

    @AutoLogOutput(key = "Shooter/Top/Velocity")
    private double getTopVelocity() {
        return topEncoder.getVelocity();
    }

    @AutoLogOutput(key = "Shooter/Bottom/Velocity")
    private double getBottomVelocity() {
        return bottomEncoder.getVelocity();
    }

    @AutoLogOutput(key = "Shooter/Velocity")
    private double getVelocity() {
        return (getTopVelocity() + getBottomVelocity()) / 2;
    }

    @AutoLogOutput(key = "Shooter/Error")
    private double getError() {
        return getVelocity() - getTargetVelocity();
    }

    @AutoLogOutput(key = "Shooter/NearTarget")
    private boolean nearTarget() {
        return Math.abs(getError()) < Constants.TOLERANCE;
    }

    @AutoLogOutput(key = "Shooter/OnTarget")
    private boolean onTarget() {
        return debouncer.calculate(nearTarget());
    }

    @Override
    public void periodic() {
        topPID.setReference(getTargetVelocity(), ControlType.kVelocity);
        bottomPID.setReference(getTargetVelocity(), ControlType.kVelocity);

        Logger.recordOutput("Shooter/Top/Output", topMotor.getAppliedOutput());
        Logger.recordOutput("Shooter/Bottom/Output", bottomMotor.getAppliedOutput());
    }

    private Command stateCommand(State target) {
        return Commands.sequence(
                runOnce(() -> state = target),
                Commands.waitSeconds(Constants.DEBOUNCE_TIME),
                run(() -> {}).until(this::onTarget));
    }

    public Command speaker() {
        return stateCommand(State.SPEAKER);
    }

    public Command amp() {
        return stateCommand(State.AMP);
    }

    public Command eject() {
        return stateCommand(State.EJECT);
    }

    public Command stop() {
        return runOnce(() -> state = State.STOPPED);
    }
}

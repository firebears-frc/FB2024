package com.seiford.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.seiford.util.spark.ClosedLoopConfiguration;
import com.seiford.util.spark.CurrentLimitConfiguration;
import com.seiford.util.spark.FeedbackConfiguration;
import com.seiford.util.spark.FollowingConfiguration;
import com.seiford.util.spark.SparkConfiguration;
import com.seiford.util.spark.StatusFrameConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    // Constants
    private static final class Constants {
        public static final int RIGHT_CAN_ID = 14;
        public static final int LEFT_CAN_ID = 15;

        public static final SparkConfiguration RIGHT_CONFIG = new SparkConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(40, 20, 10, 45.0),
                StatusFrameConfiguration.leader(),
                ClosedLoopConfiguration.simple(1.0, 0.0, 0.0, 0.0),
                FeedbackConfiguration.builtInEncoder(1));
        public static final SparkConfiguration LEFT_CONFIG = new SparkConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(40, 20, 10, 45.0),
                StatusFrameConfiguration.normal(),
                FollowingConfiguration.spark(RIGHT_CAN_ID, true));

        public static final double CLIMB_SPEED = 0.25;
        public static final double REVERSE_SPEED = -0.25;
    }

    // Objects
    private final CANSparkMax rightMotor, leftMotor;

    private double setpoint;

    // Constructor
    public Climber() {
        rightMotor = new CANSparkMax(Constants.RIGHT_CAN_ID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.LEFT_CAN_ID, MotorType.kBrushless);

        Constants.RIGHT_CONFIG.apply(rightMotor);
        Constants.LEFT_CONFIG.apply(leftMotor);

        rightMotor.burnFlash();
        leftMotor.burnFlash();
    }

    // Interface functions
    @AutoLogOutput(key = "Climber/Speed")
    private double getTargetSpeed() {
        return setpoint;
    }

    @Override
    public void periodic() {
        leftMotor.set(setpoint);
    }

    // Command factories
    public Command climb() {
        return runOnce(() -> setpoint = Constants.CLIMB_SPEED);
    }

    public Command reverse() {
        return runOnce(() -> setpoint = Constants.REVERSE_SPEED);
    }

    public Command stop() {
        return runOnce(() -> setpoint = 0.0);
    }
}

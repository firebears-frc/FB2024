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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // Constants
    private static final class Constants {
        public static final int MOTOR_CAN_ID = 9;
        public static final int SENSOR_DIO_PORT = 0;

        public static final SparkConfiguration CONFIG = new SparkConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(20, 10, 10, 25.0),
                StatusFrameConfiguration.normal(),
                ClosedLoopConfiguration.simple(0.00001, 0.0, 0.0, 0.0),
                FeedbackConfiguration.builtInEncoder(1.0));

        public static final double INTAKE_SPEED = 4000; // rotations per minute
        public static final double SHOOT_SPEED = 8000; // rotations per minute
        public static final double EJECT_SPEED = -1000; // rotations per minute

        public static final double TOLERANCE = 100; // rotations per minute
    }

    // Objects
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;
    private final DigitalInput sensor;

    private double setpoint = 0.0;
    private boolean hasNote = false;

    // Constructor
    public Intake() {
        motor = new CANSparkMax(Constants.MOTOR_CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();

        Constants.CONFIG.apply(motor);

        motor.burnFlash();

        sensor = new DigitalInput(Constants.SENSOR_DIO_PORT);
    }

    // Interface functions
    @AutoLogOutput(key = "Intake/Setpoint")
    private double getTargetVelocity() {
        return setpoint;
    }

    @AutoLogOutput(key = "Intake/Velocity")
    private double getVelocity() {
        return encoder.getVelocity();
    }

    @AutoLogOutput(key = "Intake/Error")
    private double getError() {
        return getVelocity() - getTargetVelocity();
    }

    @AutoLogOutput(key = "Intake/OnTarget")
    private boolean onTarget() {
        return Math.abs(getError()) < Constants.TOLERANCE;
    }

    @AutoLogOutput(key = "Intake/Sensor")
    private boolean getSensor() {
        return sensor.get();
    }

    @Override
    public void periodic() {
        if (getSensor() && !hasNote) {
            setpoint = 0;
            hasNote = true;
        }
        else if (!getSensor()) {
            hasNote = false;
        }

        pid.setReference(setpoint, ControlType.kVelocity);
    }

    // Command factories
    public Command shoot() {
        return runOnce(() -> setpoint = Constants.SHOOT_SPEED);
    }

    public Command intake() {
        return runOnce(() -> setpoint = Constants.INTAKE_SPEED);
    }

    public Command eject() {
        return runOnce(() -> setpoint = Constants.EJECT_SPEED);
    }

    public Command stop() {
        return runOnce(() -> setpoint = 0.0);
    }

    public Command autoIntake() {
        return run(() -> setpoint = Constants.INTAKE_SPEED).until(this::getSensor).withTimeout(2.5);
    }

    public Command intakeStop() {
        return startEnd(() -> setpoint = Constants.INTAKE_SPEED, () -> setpoint = 0.0);
    }
}

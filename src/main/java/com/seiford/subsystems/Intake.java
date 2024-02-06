package com.seiford.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.seiford.util.spark.CurrentLimitConfiguration;
import com.seiford.util.spark.SparkConfiguration;
import com.seiford.util.spark.StatusFrameConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
    // Constants
    private static final class Constants {
        public static final int MOTOR_CAN_ID = 9;
        public static final int SENSOR_DIO_PORT = 0;

        public static final SparkConfiguration CONFIG = new SparkConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(20, 10, 10, 25.0),
                StatusFrameConfiguration.normal());

        public static final double INTAKE_SPEED = 0.7; // percent
        public static final double EJECT_SPEED = -0.7; // percent
    }

    // Objects
    private final CANSparkMax motor;
    private final DigitalInput sensor;
    private final Trigger trigger;

    private double setpoint;

    // Constructor
    public Intake() {
        motor = new CANSparkMax(Constants.MOTOR_CAN_ID, MotorType.kBrushed);

        Constants.CONFIG.apply(motor);

        motor.burnFlash();

        sensor = new DigitalInput(Constants.SENSOR_DIO_PORT);
        trigger = new Trigger(this::getSensor);

        trigger.onTrue(stop());
    }

    // Interface functions
    @AutoLogOutput(key = "Intake/Speed")
    private double getTargetSpeed() {
        return setpoint;
    }

    @AutoLogOutput(key = "Intake/Sensor")
    private boolean getSensor() {
        return sensor.get();
    }

    @Override
    public void periodic() {
        motor.set(setpoint);
    }

    // Command factories
    public Command intake() {
        return runOnce(() -> setpoint = Constants.INTAKE_SPEED);
    }

    public Command eject() {
        return runOnce(() -> setpoint = Constants.EJECT_SPEED);
    }

    public Command stop() {
        return runOnce(() -> setpoint = 0.0);
    }

    public Command intakeStop() {
        return startEnd(() -> setpoint = Constants.INTAKE_SPEED, () -> setpoint = 0.0);
    }
}

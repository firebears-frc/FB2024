package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DownBeat extends SubsystemBase {
    private CANSparkMax downBeatMotor;
    private SparkPIDController pid;
    private DigitalInput sensor;

    public DownBeat() {
        downBeatMotor = new CANSparkMax(9, MotorType.kBrushless);
        downBeatMotor.setSmartCurrentLimit(10, 10);
        downBeatMotor.setSecondaryCurrentLimit(20);
        downBeatMotor.restoreFactoryDefaults();
        downBeatMotor.setInverted(false);
        downBeatMotor.setIdleMode(IdleMode.kBrake);
        pid = downBeatMotor.getPIDController();

        // set p value of pid to 1
        pid.setP(1.0);
        pid.setI(0);
        pid.setD(0);
        downBeatMotor.burnFlash();

        //sensor
        sensor = new DigitalInput(0);
        new Trigger(sensor::get).onTrue(pauseDownBeat());
    }

    public Command intakeNote() {
        return runOnce(() -> {
            pid.setReference(-0.7, ControlType.kDutyCycle);
        });
    }

    public Command dischargeNote() {
        return runOnce(() -> {
            pid.setReference(0.7, ControlType.kDutyCycle);
        });
    }

    public Command pauseDownBeat() {
        return runOnce(() -> {
            pid.setReference(0, ControlType.kDutyCycle);
        });
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Downbeat/Output", downBeatMotor.getAppliedOutput());
    }
}

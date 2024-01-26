package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class UpBeat extends SubsystemBase {
    private CANSparkMax upBeatMotor;
    private SparkPIDController pid;

    public UpBeat() {
        upBeatMotor = new CANSparkMax(10, MotorType.kBrushless);
        upBeatMotor.setSmartCurrentLimit(30, 20);
        upBeatMotor.setSecondaryCurrentLimit(40);
        upBeatMotor.restoreFactoryDefaults();
        upBeatMotor.setInverted(false);
        upBeatMotor.setIdleMode(IdleMode.kBrake);
        pid = upBeatMotor.getPIDController();

        // set p value of pid to 1
        pid.setP(6.0e-5);
        pid.setI(1.0e-6);
        pid.setD(0.0);
        pid.setFF(0.000015);
        upBeatMotor.burnFlash();
    }

    public Command shootNote() {
        return runOnce(() -> {
            pid.setReference(0.7, ControlType.kDutyCycle);
        });
    }

    public Command reverseShootNote() {
        return runOnce(() -> {
            pid.setReference(-0.7, ControlType.kDutyCycle);
        });
    }

    public Command pauseUpBeat() {
        return runOnce(() -> {
            pid.setReference(0, ControlType.kDutyCycle);
        });
    }

    @Override
    public void periodic() {
        Logger.recordOutput("upBeat/presentOutput", upBeatMotor.getAppliedOutput());
    }
}

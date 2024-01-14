package frc.robot.subsystems;

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
        upBeatMotor = new CANSparkMax(10, MotorType.kBrushed);
        upBeatMotor.setSmartCurrentLimit(10, 10);
        upBeatMotor.setSecondaryCurrentLimit(20);
        upBeatMotor.restoreFactoryDefaults();
        upBeatMotor.setInverted(false);
        upBeatMotor.setIdleMode(IdleMode.kBrake);
        pid = upBeatMotor.getPIDController();

        // set p value of pid to 1
        pid.setP(1.0);
        pid.setI(0);
        pid.setD(0);
        upBeatMotor.burnFlash();
    }

    public void shootNote() {
        pid.setReference(0.7, ControlType.kDutyCycle);
    }

    public void reverseShootNote() {
        pid.setReference(-0.7, ControlType.kDutyCycle);
    }

    public void stopUpBeat() {
        pid.setReference(0, ControlType.kDutyCycle);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("upBeat/presentOutput", upBeatMotor.getAppliedOutput());
    }
}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.AutoLogOutput;
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
    private double setPoint = 0;

    public DownBeat() {
        downBeatMotor = new CANSparkMax(9, MotorType.kBrushless);
        downBeatMotor.setSmartCurrentLimit(10, 10);
        downBeatMotor.setSecondaryCurrentLimit(20);
        downBeatMotor.restoreFactoryDefaults();
        downBeatMotor.setInverted(true);
        downBeatMotor.setIdleMode(IdleMode.kBrake);
        pid = downBeatMotor.getPIDController();

        pid.setP(0.00001);
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setFF(0.000115);
        pid.setIZone(100);
        downBeatMotor.burnFlash();

        //sensor
        sensor = new DigitalInput(0);
        new Trigger(this :: beamBreak).onTrue(pauseDownBeat());
    }

    @AutoLogOutput(key = "downBeat/beamBreak")
    private boolean beamBreak(){
        return sensor.get();
    }

    @AutoLogOutput(key = "downBeat/error")
    private double getError(){
        return setPoint-downBeatMotor.getEncoder().getVelocity();
    }

    @AutoLogOutput(key = "downBeat/atSpeed")
    private boolean atSpeed(){
        if((getError()<100)&&(getError()>-100)){
            return true;
        }
        return false;
    }

    public Command intakeNote() {
        return runOnce(() -> {
            setPoint = 7000;
        });
    }

    public Command shootNote(){
        return runOnce(() -> {
            setPoint = 8000;
        });
    }

    public Command dischargeNote() {
        return runOnce(() -> {
            setPoint = -7000;
        });
    }

    public Command pauseDownBeat() {
        return runOnce(() -> {
            setPoint = 0;
        });
    }

    @Override
    public void periodic() {
        pid.setReference(setPoint, ControlType.kVelocity);
        
        Logger.recordOutput("downBeat/Output", downBeatMotor.getAppliedOutput());
        Logger.recordOutput("downBeat/speed", downBeatMotor.getEncoder().getVelocity());
        Logger.recordOutput("downBeat/setPoint", setPoint);
    }
}

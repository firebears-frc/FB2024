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
    private CANSparkMax topMotor;
    private SparkPIDController topPid;
    private CANSparkMax bottomMotor;
    private SparkPIDController bottomPid;


    public UpBeat() {
        topMotor = new CANSparkMax(10, MotorType.kBrushless);
        topMotor.setSmartCurrentLimit(30, 20);
        topMotor.setSecondaryCurrentLimit(40);
        topMotor.restoreFactoryDefaults();
        topMotor.setInverted(false);
        topMotor.setIdleMode(IdleMode.kCoast);
        topPid = topMotor.getPIDController();

        bottomMotor = new CANSparkMax(11, MotorType.kBrushless);
        bottomMotor.setSmartCurrentLimit(30, 20);
        bottomMotor.setSecondaryCurrentLimit(40);
        bottomMotor.restoreFactoryDefaults();
        bottomMotor.setInverted(false);
        bottomMotor.setIdleMode(IdleMode.kCoast);
        bottomPid = bottomMotor.getPIDController();

        topPid.setP(6.0e-5);
        topPid.setI(1.0e-6);
        topPid.setD(0.0);
        topPid.setFF(0.000015);
        topPid.setOutputRange(0.0, 1.0);
        topMotor.burnFlash();

        bottomPid.setP(6.0e-5);
        bottomPid.setI(1.0e-6);
        bottomPid.setD(0.0);
        bottomPid.setFF(0.000015);
        bottomPid.setOutputRange(0.0, 1.0);
        bottomMotor.burnFlash();
    }

    public Command shootNote() {
        return runOnce(() -> {
            topPid.setReference(4800, ControlType.kVelocity);
            bottomPid.setReference(4000, ControlType.kVelocity);
        });
    }

    public Command reverseShootNote() {
        return runOnce(() -> {
            topPid.setReference(-1000, ControlType.kVelocity);
            bottomPid.setReference(-1200, ControlType.kVelocity);
        });
    }

    public Command pauseUpBeat() {
        return runOnce(() -> {
            topPid.setReference(0, ControlType.kVelocity);
            bottomPid.setReference(0, ControlType.kVelocity);
        });
    }

    @Override
    public void periodic() {
        Logger.recordOutput("upBeat/presentOutput", topMotor.getAppliedOutput());
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ResourceBundle.Control;

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

        topPid.setP(0.0003);
        topPid.setI(0.0000001);
        topPid.setD(0.0);
        topPid.setFF(0.0001875);
        topPid.setIZone(100);
        topPid.setOutputRange(0.0, 1.0);
        topMotor.burnFlash();

        bottomPid.setP(0.0003);
        bottomPid.setI(0.0000001);
        bottomPid.setD(0.0);
        bottomPid.setFF(0.0001875);
        bottomPid.setIZone(100);
        bottomPid.setOutputRange(0.0, 1.0);
        bottomMotor.burnFlash();
    }

    public Command shootNote() {
        return startEnd(
            () -> {
                topPid.setReference(4800, ControlType.kVelocity);
                bottomPid.setReference(4000, ControlType.kVelocity);
            },
            () -> {
                topPid.setReference(0, ControlType.kVelocity);
                bottomPid.setReference(0, ControlType.kVelocity);
            }
            );
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

    public Command ampSpeed() {
        return runOnce(() -> {
            topPid.setReference(1000, ControlType.kVelocity);
            bottomPid.setReference(1200, ControlType.kVelocity);
        });
    }

    @Override
    public void periodic() {
        Logger.recordOutput("upBeat/topOutPut", topMotor.getAppliedOutput());
        Logger.recordOutput("upBeat/ottomOutpt", bottomMotor.getAppliedOutput());
        Logger.recordOutput("upBeat/topSpeed", topMotor.getEncoder().getVelocity());
        Logger.recordOutput("upBeat/bottomSpeed", bottomMotor.getEncoder().getVelocity());
    }
}

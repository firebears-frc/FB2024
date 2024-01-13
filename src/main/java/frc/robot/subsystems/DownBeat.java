package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DownBeat extends SubsystemBase {
    private CANSparkMax downBeatMotor;
    private SparkPIDController pid;

    public DownBeat() {
        downBeatMotor = new CANSparkMax(9, MotorType.kBrushed);
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
    }
        public void intakeNote() {
            pid.setReference(0.7, ControlType.kDutyCycle);
        }

        public void dischargeNote() {
            pid.setReference(-0.7, ControlType.kDutyCycle);
        }


        public void stopDownBeat() {
            pid.setReference(0, ControlType.kDutyCycle);
        }

        @Override
        public void periodic() {
            Logger.recordOutput("downBeat/presentOutput", downBeatMotor.getAppliedOutput());
        }

        @Override
        public void simulationPeriodic() {

        }
    }

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private static int STALL_CURRENT_LIMIT_SHOULDER = 30;
    private static int FREE_CURRENT_LIMIT_SHOULDER = 25;
    private static int SECONDARY_CURRENT_LIMIT_SHOULDER = 35;

    private CANSparkMax shoulderMotorRight;
    private CANSparkMax shoulderMotorLeft;

    public Arm() {

        shoulderMotorRight = new CANSparkMax(12, MotorType.kBrushless);

        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        shoulderMotorRight.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorRight.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);

        shoulderMotorLeft = new CANSparkMax(13, MotorType.kBrushless);

        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);
        shoulderMotorLeft.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.follow(shoulderMotorRight, true);
        shoulderMotorLeft.burnFlash();
        shoulderMotorRight.burnFlash();

    }

    public void setShoulderSetpoint(double setpoint) {
        while (setpoint > 360) {
            setpoint -= 360;
        }
        while (setpoint < 0) {
            setpoint += 360;
        }

        if (setpoint < 0 || setpoint > 280) {

            setpoint = 0;
        } else if (setpoint > 130 && setpoint < 280) {

            setpoint = 130;
        } else {

        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/Shoulder/LeftOutput", shoulderMotorRight.getAppliedOutput());
        Logger.recordOutput("Arm/Shoulder/RightOutput", shoulderMotorLeft.getAppliedOutput());
    }
}
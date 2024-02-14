package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Glissando extends SubsystemBase {
    private CANSparkMax climbRight;
    private CANSparkMax climbLeft;

    private static int stallLimit = 30;
    private static int freeLimit = 20;
    private static int scndLimit = 40;

    private static double climbSpeed = 0.25;

    private double setPoint = 0;

    public Glissando() {
        climbRight = new CANSparkMax(14, MotorType.kBrushless);
        climbLeft = new CANSparkMax(15, MotorType.kBrushless);

        climbRight.setInverted(false);
        climbLeft.setInverted(true);

        climbRight.setIdleMode(IdleMode.kBrake);
        climbLeft.setIdleMode(IdleMode.kBrake);

        climbRight.setSmartCurrentLimit(stallLimit, freeLimit);
        climbLeft.setSmartCurrentLimit(stallLimit, freeLimit);

        climbRight.setSecondaryCurrentLimit(scndLimit);
        climbLeft.setSecondaryCurrentLimit(scndLimit);
    }

    public Command climb() {
        return runOnce(() -> {
            setPoint = climbSpeed;
        });
    }

    public Command unclimb() {
        return runOnce(() -> {
            setPoint = -(climbSpeed);
        });
    }

    public Command pauseClimb() {
        return runOnce(() -> {
            setPoint = 0;
        });
    }

    @Override
    public void periodic(){
        climbLeft.set(setPoint);
        climbRight.set(setPoint);

        Logger.recordOutput("Glissando/setPoint", setPoint);
    }
}

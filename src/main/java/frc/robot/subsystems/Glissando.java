package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Glissando extends SubsystemBase {
    private CANSparkMax climbRight;
    private CANSparkMax climbLeft;

    private static int stallLimit = 30;
    private static int freeLimit = 20;
    private static int scndLimit = 40;

    private static double climbSpeed = 1;

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

        climbRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        climbRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        climbRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
        climbLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        climbLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        climbLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

        climbRight.burnFlash();
        climbLeft.burnFlash();
    }

    public Command climb() {
        return runOnce(() -> {
            climbRight.set(climbSpeed);
            climbLeft.set(climbSpeed);
        });
    }

    public Command unclimb() {
        return runOnce(() -> {
            climbRight.set(-(climbSpeed));
            climbLeft.set(-(climbSpeed));
        });
    }

    public Command pauseClimb() {
        return runOnce(() -> {
            climbRight.set(0);
            climbLeft.set(0);
        });
    }

    public Command climbHalfSpeed(){
            return runOnce(() -> {
                climbRight.set((climbSpeed)/2);
                climbLeft.set((climbSpeed)/2);
            });
    }

    public Command climbRightUp(){
        return runOnce(() -> {
            climbRight.set(climbSpeed);
        });

    }
    public Command climbRightDown(){
        return runOnce(() -> {
            climbRight.set(-(climbSpeed));
        });
    }

    public Command climbLeftUp(){
        return runOnce(() -> {
            climbLeft.set(climbSpeed);
        });

    }
    public Command climbLeftDown(){
        return runOnce(() -> {
            climbLeft.set(-(climbSpeed));
        });
    }
}

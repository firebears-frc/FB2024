package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Glissando extends SubsystemBase {
    private CANSparkMax climbRight;
    private CANSparkMax climbLeft;

    private AHRS gyro;

    private static int stallLimit = 30;
    private static int freeLimit = 20;
    private static int scndLimit = 40;

    private double rightCommandSpeed;
    private double leftCommandSpeed;
    private double rightClimbSpeed;
    private double leftClimbSpeed;

    public Glissando() {
        gyro = new AHRS(SPI.Port.kMXP);

        rightCommandSpeed = 0;
        leftCommandSpeed = 0;
        rightClimbSpeed = 0;
        leftClimbSpeed = 0;

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
            rightCommandSpeed = 1;
            leftCommandSpeed = .7;
        });
    }

    public Command unclimb() {
        return runOnce(() -> {
            rightCommandSpeed = -1;
            leftCommandSpeed = -.7;
        });
    }

    public Command pauseClimb() {
        return runOnce(() -> {
            rightCommandSpeed = 0;
            leftCommandSpeed = 0;
        });
    }

    public Command climbHalfSpeed(){
            return runOnce(() -> {
            rightCommandSpeed = 0.5;
            leftCommandSpeed = 0.5;
            });
    }

    public Command climbRightUp(){
        return runOnce(() -> {
            rightCommandSpeed = 1;
        });

    }
    public Command climbRightDown(){
        return runOnce(() -> {
            rightCommandSpeed = -1;
        });
    }

    public Command climbLeftUp(){
        return runOnce(() -> {
            leftCommandSpeed = 1;
        });

    }
    public Command climbLeftDown(){
        return runOnce(() -> {
            leftCommandSpeed = -1;
        });
    }

    private void level(){
        if(gyro.getRoll()<=-5){
            rightClimbSpeed = rightCommandSpeed*0.25;
            leftClimbSpeed = leftCommandSpeed;
        }else if(gyro.getRoll()>=5){
            rightClimbSpeed = rightCommandSpeed;
            leftClimbSpeed = leftCommandSpeed*0.25;
        }else{
            rightClimbSpeed = rightCommandSpeed;
            leftClimbSpeed = leftCommandSpeed;
        }
    }

    @Override
    public void periodic() {
        level();

        climbRight.set(rightClimbSpeed);
        climbLeft.set(leftClimbSpeed);

        Logger.recordOutput("glissando/setPointRight", rightCommandSpeed);
        Logger.recordOutput("glissando/setPointLeft", leftCommandSpeed);        
        Logger.recordOutput("glissando/climbRight", rightClimbSpeed);
        Logger.recordOutput("glissando/climbLeft", leftClimbSpeed);
        Logger.recordOutput("glissando/gyro", gyro.getRoll());
    }
}
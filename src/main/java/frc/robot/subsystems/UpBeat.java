package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class UpBeat extends SubsystemBase {
    private CANSparkMax topMotor;
    private SparkPIDController topPid;
    private CANSparkMax bottomMotor;
    private SparkPIDController bottomPid;
    @AutoLogOutput(key = "upBeat/setPoint")
    private double setPoint = 0;


    public UpBeat() {
        topMotor = new CANSparkMax(10, MotorType.kBrushless);
        topMotor.setSmartCurrentLimit(50, 50);
        topMotor.setSecondaryCurrentLimit(60);
        topMotor.restoreFactoryDefaults();
        topMotor.setInverted(false);
        topMotor.setIdleMode(IdleMode.kCoast);
        topPid = topMotor.getPIDController();
        topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

        bottomMotor = new CANSparkMax(11, MotorType.kBrushless);
        bottomMotor.setSmartCurrentLimit(50, 50);
        bottomMotor.setSecondaryCurrentLimit(60);
        bottomMotor.restoreFactoryDefaults();
        bottomMotor.setInverted(false);
        bottomMotor.setIdleMode(IdleMode.kCoast);
        bottomPid = bottomMotor.getPIDController();
        bottomMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        bottomMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        bottomMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

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

    private final static class Constants{
        private static final double stop = 0.00;
        private static final double reverse = -1000.00;
        private static final double shoot = 3600.00;
        private static final double amp = 1000.00;
        private static final double straightShot = 3600.00;
    }

    @AutoLogOutput(key = "upBeat/speed")
    private double getSpeed() {
        return bottomMotor.getEncoder().getVelocity();
    }

    @AutoLogOutput(key = "upBeat/error")
    private double getError(){
        return setPoint-getSpeed();
    }

    @AutoLogOutput(key = "upBeat/at speed")
    private boolean atSpeed(){
        return Math.abs(getError()) < 100;
    }

    private Command speedCommand(double speed){
        return run (()-> setPoint = speed).until(this::atSpeed); 
    }

    public Command shootNote() {
        return startEnd(
            () -> setPoint = Constants.shoot,
            () -> setPoint = Constants.stop
            );
    }

    public Command reverseShootNote() {
        return speedCommand(Constants.reverse);
    }

    public Command pauseUpBeat() {
        return speedCommand(Constants.stop);
    }

    public Command ampSpeed() {
        return speedCommand(Constants.amp);
    }

    public Command autoShoot() {
        return speedCommand(Constants.shoot);
    }

    public Command straightAutoShot(){
        return speedCommand(Constants.straightShot);
    }

    @Override
    public void periodic() {
        topPid.setReference(setPoint, ControlType.kVelocity);
        bottomPid.setReference(setPoint, ControlType.kVelocity);

        Logger.recordOutput("upBeat/topOutput", topMotor.getAppliedOutput());
        Logger.recordOutput("upBeat/bottomOutput", bottomMotor.getAppliedOutput());
        Logger.recordOutput("upBeat/topSpeed", topMotor.getEncoder().getVelocity());
        Logger.recordOutput("upBeat/bottomSpeed", bottomMotor.getEncoder().getVelocity());
    }
}

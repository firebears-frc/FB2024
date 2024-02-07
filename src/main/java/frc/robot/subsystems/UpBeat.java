package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ResourceBundle.Control;

import org.littletonrobotics.junction.AutoLogOutput;
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
    @AutoLogOutput(key = "upBeat/setPoint")
    private double setPoint = 0;


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

    private final static class Constants{
        private static final double stop = 0.00;
        private static final double reverse = -1000.00;
        private static final double shoot = 4000.00;
        private static final double amp = 1000.00;
    }

    @AutoLogOutput(key = "upBeat/speed")
    private double getSpeed() {
        return bottomMotor.getEncoder().getVelocity();
    }

    @AutoLogOutput(key = "upBeat/error")
    private double getError(){
        return setPoint-getSpeed();
    }

    private boolean atSpeed(){
        if(getError()<-100 && getError()>100){
        return true;
        }
        return false;
    }

    private void setSpeed(){
        topPid.setReference((setPoint*1.2), ControlType.kVelocity);
        bottomPid.setReference(setPoint, ControlType.kVelocity);
    }

    private Command speedCommand(double speed){
        setPoint = speed;
        return run (()-> setSpeed()).until(this::atSpeed); 
    }

    public Command shootNote() {
        return startEnd(
            () -> {
                speedCommand(Constants.shoot);
            },
            () -> {
                speedCommand(Constants.stop);
            }
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

    @Override
    public void periodic() {
        Logger.recordOutput("upBeat/topOutPut", topMotor.getAppliedOutput());
        Logger.recordOutput("upBeat/ottomOutpt", bottomMotor.getAppliedOutput());
        Logger.recordOutput("upBeat/topSpeed", topMotor.getEncoder().getVelocity());
        Logger.recordOutput("upBeat/bottomSpeed", bottomMotor.getEncoder().getVelocity());
    }
}

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.ResourceBundle.Control;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
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
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.25044, 0.00039235);


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

        topPid.setP(3.0156E-07);
        topPid.setI(0.0);
        topPid.setD(0.0);
        topPid.setFF(0.0001875);
        topPid.setIZone(100);
        topPid.setOutputRange(0.0, 1.0);
        topMotor.burnFlash();

        bottomPid.setP(3.0156E-07);
        bottomPid.setI(0.0);
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

    @AutoLogOutput(key = "upBeat/at speed")
    private boolean atSpeed(){
        return Math.abs(getError()) < 100;
    }

    private Command speedCommand(double speed){
        return run (()-> setPoint = speed).until(this::atSpeed); 
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
        double feedforwardVolts = feedforward.calculate(getSpeed(), setPoint, LoggedRobot.defaultPeriodSecs);
        topPid.setReference(setPoint, ControlType.kVelocity, 0, feedforwardVolts);
        bottomPid.setReference(setPoint, ControlType.kVelocity, 0, feedforwardVolts);

        Logger.recordOutput("upBeat/feedForward", feedforwardVolts);
        Logger.recordOutput("upBeat/topOutput", topMotor.getAppliedOutput());
        Logger.recordOutput("upBeat/bottomOutput", bottomMotor.getAppliedOutput());
        Logger.recordOutput("upBeat/topSpeed", topMotor.getEncoder().getVelocity());
        Logger.recordOutput("upBeat/bottomSpeed", bottomMotor.getEncoder().getVelocity());
    }
}

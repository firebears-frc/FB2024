package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private CANSparkMax topMotor;//these are all vars: topMotor, topPid, bottomMotor, bottomPid, setpoint and debounce
    private SparkPIDController topPid;//they are also private similar to the constants bellow
    private CANSparkMax bottomMotor;
    private SparkPIDController bottomPid;
    @AutoLogOutput(key = "upBeat/setPoint")
    private double setPoint = 0;//setpoint is a var and unlike a constant vars can be changed this is what we set to = the constants from before
                                //you can also add to vars together think of them like x or y from math if x=2 and y=3 and x+y=5 
                                //now say x = setpoint what does y+x=?
    private Debouncer debounce = new Debouncer(0.2);//Move down to line 98

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

    private final static class Constants { //These are the constants indicated by the word final. shooter stop, reverse, shoot, amp, straightShot, and magicNumber
        private static final double stop = 0.00;//all of them are a number and this number is used to tell motors what rpm they should spin at
        private static final double reverse = -1000.00;//after a constant is declared it cant be changed. 
        private static final double shoot = 3600.00;//Constants are usefull because you might use the same number multiple times but if that number needs to be change you only have to change it here
        private static final double amp = 1000.00;//all of them are also private indicates that the const can only be used were the constant was declared in this case thats theUpBeat 
        private static final double straightShot = 3600.00;
        private static final double magicNumber = 5000.0;//Move up to line 19
    }

    @AutoLogOutput(key = "upBeat/speed")
    private double getSpeed() {
        return bottomMotor.getEncoder().getVelocity();
    }

    @AutoLogOutput(key = "upBeat/error")
    private double getError() {//
        return setPoint - getSpeed();
    }

    @AutoLogOutput(key = "upBeat/at speed")
    private boolean atSpeed() {
        return Math.abs(getError()) < 100;
    }

    @AutoLogOutput(key = "upBeat/at debouncespeed")
    private boolean debounceSpeend() {
        return debounce.calculate(atSpeed());
    }

    private Command speedCommand(double speed) {//this is an example of a Command. Notice there is an input speed on line 98 and on 99 it returns commands.sequence
                                                //the bounding box of what is in and outside the command are determined by the { } on line 98 and line 108
                                                //think of this as f(x) were f is speedCommand and x is speed. if f(x)=x+1 than f(2)=3 this is the same
                                                //speedCommand(speed) sets the var setPoint to speed then waits 0.1 second then runs another command
                                                //Move down to line 123                
        return Commands.sequence(               
            runOnce(() -> setPoint = speed),    
            Commands.waitSeconds(0.1),
            run(() -> {}).until(this::debounceSpeend)
        );
    }

    public Command shootNote() {
        return startEnd(
                () -> setPoint = Constants.magicNumber,
                () -> setPoint = Constants.stop);
    }

    public Command reverseShootNote() {
        return speedCommand(Constants.reverse);
    }

    public Command pauseUpBeat() {
        return speedCommand(Constants.stop);
    }

    public Command ampSpeed() {// here we use that speed command we input constants.amp from before and it will return changiong that setpoint to constants.amp
        return speedCommand(Constants.amp);//if this command sets setpoint to constants.amp what is the value of setPoint right now?
    }                                      //move to line 136

    public Command autoShoot() {
        return speedCommand(Constants.shoot);
    }

    public Command straightAutoShot() {
        return speedCommand(Constants.straightShot);
    }

    @Override
    public void periodic() {                                  //Here is were we finaly tell the motor to run periodic we will go into later but think of it as an event scheduler           
        topPid.setReference(setPoint, ControlType.kVelocity); //it will run everything in it every couple of milliseconds. topPid and BottomPid will also be for later but it in general
        bottomPid.setReference(setPoint, ControlType.kVelocity);//all motor on our robot are controled by a compplicated thing called PID  

        Logger.recordOutput("upBeat/topOutput", topMotor.getAppliedOutput());// thats the end now if there is time I would like everyone to go to either the downbeat the arm or the bass as a challange
        Logger.recordOutput("upBeat/bottomOutput", bottomMotor.getAppliedOutput());//and lable a var const and a command wit there name. Please use comments like i have. type // and it will alow you to write 
        Logger.recordOutput("upBeat/topSpeed", topMotor.getEncoder().getVelocity());// without messing up the code. once down labling i got nothing else for you.
        Logger.recordOutput("upBeat/bottomSpeed", bottomMotor.getEncoder().getVelocity());
    }
}

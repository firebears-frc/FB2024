package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants.*;

public class Arm extends SubsystemBase {
    private static int STALL_CURRENT_LIMIT_SHOULDER = 30;
    private static int FREE_CURRENT_LIMIT_SHOULDER = 25;
    private static int SECONDARY_CURRENT_LIMIT_SHOULDER = 35;
    private CANSparkMax shoulderMotorRight;
    private CANSparkMax shoulderMotorLeft;
    private static SparkAbsoluteEncoder shoulderEncoder;
    private SparkPIDController shoulderPID;
    @AutoLogOutput(key = "arm/setPoint")
    private Rotation2d shoulderSetpoint = new Rotation2d();

    public Arm() {
        shoulderMotorRight = new CANSparkMax(13, MotorType.kBrushless);

        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        shoulderMotorRight.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorRight.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
        shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        shoulderMotorLeft = new CANSparkMax(12, MotorType.kBrushless);

        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);
        shoulderMotorLeft.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.follow(shoulderMotorRight, true);
        shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        shoulderMotorLeft.burnFlash();

        shoulderPID = shoulderMotorRight.getPIDController();
        shoulderEncoder = shoulderMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderEncoder.setInverted(true);
        shoulderPID.setP(ArmConstants.shoulderP);
        shoulderPID.setI(ArmConstants.shoulderI);
        shoulderPID.setD(ArmConstants.shoulderD);

        shoulderPID.setFeedbackDevice(shoulderEncoder);
        shoulderPID.setPositionPIDWrappingEnabled(true);
        shoulderPID.setPositionPIDWrappingMinInput(0.0);
        shoulderPID.setPositionPIDWrappingMaxInput(360);
        shoulderEncoder.setPositionConversionFactor(360);
        shoulderMotorRight.burnFlash();

        setShoulderSetpoint(getShoulderAngle());
    }

    private final static class Constants{     // arm setpoints
        private static final Rotation2d pickUp = Rotation2d.fromDegrees(0);
        private static final Rotation2d speakerShoot = Rotation2d.fromDegrees(14.5);
        private static final Rotation2d ampShoot = Rotation2d.fromDegrees(85);
        private static final Rotation2d stow = Rotation2d.fromDegrees(20);
        private static final Rotation2d sideShoot = Rotation2d.fromDegrees(30);
        
    }
    @AutoLogOutput(key = "arm/Angle")
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromDegrees(shoulderEncoder.getPosition());
    }
    
    public void setShoulderSetpoint(Rotation2d setpoint) {        
        if (setpoint.getDegrees() < -5) {
             setpoint = Rotation2d.fromDegrees(-5);
        } else if (setpoint.getDegrees() > 130) {
             setpoint = Rotation2d.fromDegrees(130);
        }
        shoulderSetpoint = setpoint;
    }
    @AutoLogOutput(key = "arm/error")
    private Rotation2d getError(){
        return getShoulderAngle().minus(shoulderSetpoint);
    }

    @AutoLogOutput(key = "arm/onTarget")
    private boolean onTarget(){
      return Math.abs(getError().getDegrees()) < 1;

    }

    public Command defaultCommand(Supplier<Double> shoulderChange) {
        return run(() -> {
           setShoulderSetpoint(shoulderSetpoint.minus(Rotation2d.fromDegrees(shoulderChange.get())));
        });
    }
    
    public Command pickUp(){
        return positionCommand(Constants.pickUp);
    }
    public Command speakerShoot(){
        return positionCommand(Constants.speakerShoot);
    }

    public Command ampShoot(){
        return positionCommand(Constants.ampShoot);
    }

    public Command stow(){
        return positionCommand(Constants.stow);
    }
    public Command sideShoot(){
        return positionCommand(Constants.sideShoot);
    }

    private Command positionCommand(Rotation2d position){
        return run(()-> setShoulderSetpoint(position)).until(this::onTarget);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("arm/MotorLeft", shoulderMotorLeft.getAppliedOutput());
        Logger.recordOutput("arm/MotorRight", shoulderMotorRight.getAppliedOutput());    
        Logger.recordOutput("arm/setPointDegrees", shoulderSetpoint.getDegrees());
        shoulderPID.setReference(shoulderSetpoint.getDegrees(), ControlType.kPosition);
    }
    
}


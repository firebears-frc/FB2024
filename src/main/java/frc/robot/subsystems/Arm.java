package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private static int STALL_CURRENT_LIMIT_SHOULDER = 20;
    private static int FREE_CURRENT_LIMIT_SHOULDER = 20;
    private static int SECONDARY_CURRENT_LIMIT_SHOULDER = 30;
    private CANSparkMax shoulderMotorRight;
    private CANSparkMax shoulderMotorLeft;
    private static SparkAbsoluteEncoder shoulderEncoder;
    private SparkPIDController shoulderPID;
    @AutoLogOutput(key = "arm/setPoint")
    private Rotation2d shoulderSetpoint = new Rotation2d();
    private Debouncer debounce = new Debouncer(0.2);

    private final Supplier<Rotation2d> angleSupplier;

    public Arm(Supplier<Rotation2d> angSupplier) {
        shoulderMotorRight = new CANSparkMax(13, MotorType.kBrushless);

        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        shoulderMotorRight.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorRight.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
        shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

        shoulderMotorLeft = new CANSparkMax(12, MotorType.kBrushless);

        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);
        shoulderMotorLeft.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.follow(shoulderMotorRight, true);
        shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
        shoulderMotorLeft.burnFlash();

        this.angleSupplier = angSupplier;

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
        private static final Rotation2d speakerShoot = Rotation2d.fromDegrees(13.5);
        private static final Rotation2d ampShoot = Rotation2d.fromDegrees(90);
        private static final Rotation2d stow = Rotation2d.fromDegrees(20);
        private static final Rotation2d sideShoot = Rotation2d.fromDegrees(33.75);
        private static final Rotation2d straightShot = Rotation2d.fromDegrees(14.5);
        
    }
    @AutoLogOutput(key = "arm/Angle")
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromDegrees(shoulderEncoder.getPosition());
    }
    
    public void setShoulderSetpoint(Rotation2d setpoint) {        
        if (setpoint.getDegrees() < -5) {
             setpoint = Rotation2d.fromDegrees(-5);
        } else if (setpoint.getDegrees() > 100) {
             setpoint = Rotation2d.fromDegrees(100);
        }
        shoulderSetpoint = setpoint;
    }
    @AutoLogOutput(key = "arm/error")
    private Rotation2d getError(){
        return getShoulderAngle().minus(shoulderSetpoint);
    }

    public Command defaultCommand(Supplier<Double> shoulderChange) {
        return run(() -> {
           setShoulderSetpoint(shoulderSetpoint.minus(Rotation2d.fromDegrees(shoulderChange.get())));
        });
    }

    public Command pickUp(){
        return positionCommand(Constants.pickUp, 1);
    }

    public Command speakerShoot(){
        return positionCommand(Constants.speakerShoot, 1);
    }

    public Command ampShoot(){
        return positionCommand(Constants.ampShoot, 1);
    }

    public Command sideShoot(){
        return positionCommand(Constants.sideShoot, 1);
    }
    public Command straightShot(){
        return positionCommand(Constants.straightShot, 1);
    }


    private boolean onTarget(double tolerance){
        boolean onTarget = Math.abs(getError().getDegrees()) < tolerance;
        Logger.recordOutput("arm/onTargt",onTarget);
        boolean debounced = debounce.calculate(onTarget);
        Logger.recordOutput("arm/at debouncespeed",debounced);
        return debounced;
    }

    private Command positionCommand(Rotation2d position, double tolerance){
        return Commands.sequence(
            runOnce(() -> setShoulderSetpoint(position)),
            Commands.waitSeconds(0.1),
            run(()-> {}).until(()-> onTarget(tolerance))
        );
    }

     public Command groundSlam(){
        return Commands.sequence(
            runOnce(() -> setShoulderSetpoint(Constants.pickUp)),
            Commands.waitSeconds(0.1),
            run(()-> {}).until(()-> getShoulderAngle().getDegrees() < 5)
        );
    }

    @Override
    public void periodic() {
        double feedForward = Math.cos(getShoulderAngle().getRadians()) * ArmConstants.shoulderG;
        shoulderPID.setReference(shoulderSetpoint.getDegrees(), ControlType.kPosition, 0, feedForward);

        Logger.recordOutput("arm/MotorLeft", shoulderMotorLeft.getAppliedOutput());
        Logger.recordOutput("arm/MotorRight", shoulderMotorRight.getAppliedOutput());
        Logger.recordOutput("arm/MotorLeftCurrent", shoulderMotorLeft.getOutputCurrent());
        Logger.recordOutput("arm/MotorRightCurrent", shoulderMotorRight.getOutputCurrent());
        Logger.recordOutput("arm/setPointDegrees", shoulderSetpoint.getDegrees());
        Logger.recordOutput("arm/FeedForward", feedForward);
    }    
}

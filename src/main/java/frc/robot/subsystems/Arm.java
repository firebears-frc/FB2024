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
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants.*;

public class Arm extends SubsystemBase {
    private static int STALL_CURRENT_LIMIT_SHOULDER = 30;
    private static int FREE_CURRENT_LIMIT_SHOULDER = 25;
    private static int SECONDARY_CURRENT_LIMIT_SHOULDER = 35;
    private CANSparkMax shoulderMotorRight;
    private CANSparkMax shoulderMotorLeft;
    private static SparkAbsoluteEncoder shoulderEncoderRight;
    private static SparkAbsoluteEncoder shoulderEncoderLeft;
    private SparkPIDController shoulderRightPID;
    private SparkPIDController shoulderLeftPID;
    @AutoLogOutput(key = "Arm/Setpoint")
    private Rotation2d shoulderSetpoint = new Rotation2d();

    public Arm() {
        shoulderMotorRight = new CANSparkMax(13, MotorType.kBrushless);

        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        shoulderMotorRight.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorRight.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
        shoulderMotorRight.burnFlash();

        shoulderMotorLeft = new CANSparkMax(12, MotorType.kBrushless);

        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);
        shoulderMotorLeft.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.burnFlash();

        shoulderRightPID = shoulderMotorRight.getPIDController();
        shoulderEncoderRight = shoulderMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderEncoderRight.setInverted(true);
        shoulderRightPID.setP(ArmConstants.shoulderP);
        shoulderRightPID.setI(ArmConstants.shoulderI);
        shoulderRightPID.setD(ArmConstants.shoulderD);

        shoulderRightPID.setFeedbackDevice(shoulderEncoderRight);
        shoulderRightPID.setPositionPIDWrappingEnabled(true);
        shoulderRightPID.setPositionPIDWrappingMinInput(0.0);
        shoulderRightPID.setPositionPIDWrappingMaxInput(360);
        shoulderEncoderRight.setPositionConversionFactor(360);
        shoulderMotorRight.burnFlash();

        shoulderLeftPID = shoulderMotorLeft.getPIDController();
        shoulderEncoderLeft = shoulderMotorLeft.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderEncoderLeft.setInverted(true);
        shoulderLeftPID.setP(ArmConstants.shoulderP);
        shoulderLeftPID.setI(ArmConstants.shoulderI);
        shoulderLeftPID.setD(ArmConstants.shoulderD);

        shoulderLeftPID.setFeedbackDevice(shoulderEncoderLeft);
        shoulderLeftPID.setPositionPIDWrappingEnabled(true);
        shoulderLeftPID.setPositionPIDWrappingMinInput(0.0);
        shoulderLeftPID.setPositionPIDWrappingMaxInput(360);
        shoulderEncoderLeft.setPositionConversionFactor(360);
        shoulderMotorLeft.burnFlash();

    }

    private final static class Constants{     // arm setpoints
        private static final Rotation2d pickUp = Rotation2d.fromDegrees(-3);
        private static final Rotation2d speakerShoot = Rotation2d.fromDegrees(6.5);
        private static final Rotation2d ampShoot = Rotation2d.fromDegrees(85);
        private static final Rotation2d stow = Rotation2d.fromDegrees(20);
    }
    @AutoLogOutput(key = "Arm/Left/Actual")
    public Rotation2d getLeftShoulderAngle() {
        return Rotation2d.fromDegrees(shoulderEncoderLeft.getPosition());
    }

    @AutoLogOutput(key = "Arm/Right/Actual")
    public Rotation2d getRightShoulderAngle() {
        return Rotation2d.fromDegrees(shoulderEncoderRight.getPosition());
    }
    
    public void setShoulderSetpoint(Rotation2d setpoint) {        
        if (setpoint.getDegrees() < -5) {
             setpoint = Rotation2d.fromDegrees(-5);
        } else if (setpoint.getDegrees() > 130) {
             setpoint = Rotation2d.fromDegrees(130);
        }
        shoulderSetpoint = setpoint;
    }
    @AutoLogOutput(key = "Arm/Left/Error")
    private Rotation2d getLeftError(){
        return getLeftShoulderAngle().minus(shoulderSetpoint);
    }

    @AutoLogOutput(key = "Arm/Right/Error")
    private Rotation2d getRightError(){
        return getRightShoulderAngle().minus(shoulderSetpoint);
    }

    @AutoLogOutput(key = "Arm/OnTarget")
    private boolean onTarget(){
      return Math.abs(getLeftError().getDegrees()) < 2 && Math.abs(getRightError().getDegrees()) < 2;

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

    private Command positionCommand(Rotation2d position){
        return run(()-> setShoulderSetpoint(position)).until(this::onTarget);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/Left/Output", shoulderMotorLeft.getAppliedOutput());
        Logger.recordOutput("Arm/Right/Output", shoulderMotorRight.getAppliedOutput());    
        Logger.recordOutput("Arm/SetpointDegrees", shoulderSetpoint.getDegrees());

        shoulderRightPID.setReference(shoulderSetpoint.getDegrees(), ControlType.kPosition);
        shoulderLeftPID.setReference(shoulderSetpoint.getDegrees(), ControlType.kPosition);
    }
    
}

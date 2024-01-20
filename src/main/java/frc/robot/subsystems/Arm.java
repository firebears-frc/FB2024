package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import static frc.robot.Constants.*;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private static int STALL_CURRENT_LIMIT_SHOULDER = 30;
    private static int FREE_CURRENT_LIMIT_SHOULDER = 25;
    private static int SECONDARY_CURRENT_LIMIT_SHOULDER = 35;

    private CANSparkMax shoulderMotorRight;
    private CANSparkMax shoulderMotorLeft;
    private static SparkAbsoluteEncoder shoulderEncoder;
    private SparkPIDController shoulderPID;
    private double shoulderSetpoint;
    public double ARM_SHOULDER_LENGTH;

    public Arm() {

        shoulderMotorRight = new CANSparkMax(8, MotorType.kBrushless);

        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        shoulderMotorRight.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorRight.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);

        shoulderMotorLeft = new CANSparkMax(9, MotorType.kBrushless);

        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);
        shoulderMotorLeft.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.follow(shoulderMotorRight, true);
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

    }

    public double getShoulderAngle() {
        double angle = shoulderEncoder.getPosition();
        return angle;
    }

    public void setShoulderSetpoint(double setpoint) {
        while (setpoint > 360) {
            setpoint -= 360;
        }
        while (setpoint < 0) {
            setpoint += 360;
        }

        if (setpoint < 0 || setpoint > 280) {

            setpoint = 0;
        } else if (setpoint > 130 && setpoint < 280) {

            setpoint = 130;
        } else {

        }
        if (!violatesFramePerimeter( getShoulderAngle())) {
            shoulderSetpoint = setpoint;

        } else {
            System.out.println("hit limit");
        }
    }

    public double getShoulderSetpoint() {
        return shoulderSetpoint;
    }

    public Translation2d getArmPosition(double shoulder_Angle) {
        // 105 is when its 6 inches off

        double shoulder_Compliment = 180 - shoulder_Angle;

        Translation2d output = new Translation2d();
        return output;
    }

    public boolean violatesFramePerimeter(double shoulder_Angle) {
        double currentExtension = getArmPosition(getShoulderAngle()).getX();
        double desiredExtension = getArmPosition(shoulder_Angle).getX();
        return shoulder_Angle > 105 && !(desiredExtension < currentExtension || desiredExtension < 45);

    }

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/Shoulder/Angle", getShoulderAngle());
        Logger.recordOutput("Arm/Shoulder/Setpoint", shoulderSetpoint);
        Logger.recordOutput("Arm/Shoulder/LeftOutput", shoulderMotorRight.getAppliedOutput());
        Logger.recordOutput("Arm/Shoulder/RightOutput", shoulderMotorLeft.getAppliedOutput());
        Logger.recordOutput("Arm/X", getArmPosition(getShoulderAngle()).getX());

        shoulderPID.setReference(shoulderSetpoint, ControlType.kPosition);
    }
}
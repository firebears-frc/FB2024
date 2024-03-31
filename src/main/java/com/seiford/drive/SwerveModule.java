package com.seiford.drive;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.seiford.util.spark.ClosedLoopConfiguration;
import com.seiford.util.spark.CurrentLimitConfiguration;
import com.seiford.util.spark.FeedbackConfiguration;
import com.seiford.util.spark.SparkConfiguration;
import com.seiford.util.spark.StatusFrameConfiguration;

public class SwerveModule {
    private static final class Constants {
        public static final double NEO_VORTEX_FREE_SPEED = 6784.0 / 60; // rotations per second

        private static class Driving {
            private static final double WHEEL_DIAMETER = 0.07331075; // meters
            private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // meters
            // 12T, 13T, 14T, 15T, or 16T gearing
            private static final int PINION_TEETH = 16;
            // 22T, 21T, 20T, or 19T gearing
            private static final int SPUR_TEETH = 20;
            private static final double GEAR_RATIO = 45.0 * SPUR_TEETH / (PINION_TEETH * 15);
            private static final double FREE_SPEED = NEO_VORTEX_FREE_SPEED * WHEEL_CIRCUMFERENCE / GEAR_RATIO; // meters
                                                                                                               // per
                                                                                                               // second
            public static final double POSITION_FACTOR = WHEEL_CIRCUMFERENCE / GEAR_RATIO; // meters

            public static final SparkConfiguration CONFIG = new SparkConfiguration(
                    false,
                    IdleMode.kBrake,
                    CurrentLimitConfiguration.complex(50, 20, 10, 60.0),
                    StatusFrameConfiguration.normal(),
                    ClosedLoopConfiguration.simple(0.25, 0.0, 0.0, 1.0 / FREE_SPEED),
                    FeedbackConfiguration.builtInEncoder(POSITION_FACTOR));
        }

        private static class Turning {
            public static final SparkConfiguration CONFIG = new SparkConfiguration(
                    false,
                    IdleMode.kBrake,
                    CurrentLimitConfiguration.complex(20, 10, 10, 30.0),
                    StatusFrameConfiguration.absoluteEncoder(),
                    ClosedLoopConfiguration.wrapping(3.5, 0.0, 0.0, 0.0, 0, 360),
                    FeedbackConfiguration.absoluteEncoder(true, 360));
        }
    }

    private final CANSparkMax drivingMotor;
    private final RelativeEncoder drivingEncoder;
    private final SparkPIDController drivingController;

    private final CANSparkMax turningMotor;
    private final AbsoluteEncoder turningEncoder;
    private final SparkPIDController turningController;

    private final Rotation2d angleOffset;
    private final String name;

    @AutoLogOutput(key = "Drive/Modules/{name}/Target")
    private SwerveModuleState desiredState;

    public SwerveModule(SwerveModuleConfiguration configuration) {
        drivingMotor = new CANSparkMax(configuration.drivingID, MotorType.kBrushless);
        drivingEncoder = drivingMotor.getEncoder();
        drivingController = drivingMotor.getPIDController();

        turningMotor = new CANSparkMax(configuration.turningID, MotorType.kBrushless);
        turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
        turningController = turningMotor.getPIDController();

        Constants.Driving.CONFIG.apply(drivingMotor);
        Constants.Turning.CONFIG.apply(turningMotor);

        drivingEncoder.setPosition(0);
        angleOffset = configuration.angleOffset;
        name = configuration.name;
        desiredState = new SwerveModuleState(0.0, new Rotation2d(turningEncoder.getPosition()));

        drivingMotor.burnFlash();
        turningMotor.burnFlash();
    }

    public void setDesiredState(SwerveModuleState state) {
        state.angle = state.angle.plus(angleOffset);
        state = SwerveModuleState.optimize(state, new Rotation2d(turningEncoder.getPosition()));

        drivingController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turningController.setReference(state.angle.getDegrees(), CANSparkMax.ControlType.kPosition);

        desiredState = state;
    }

    @AutoLogOutput(key = "Drive/Modules/{name}/Position")
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                drivingEncoder.getPosition(),
                Rotation2d.fromDegrees(turningEncoder.getPosition()).minus(angleOffset));
    }

    @AutoLogOutput(key = "Drive/Modules/{name}/Actual")
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                drivingEncoder.getVelocity(),
                Rotation2d.fromDegrees(turningEncoder.getPosition()).minus(angleOffset));
    }
}
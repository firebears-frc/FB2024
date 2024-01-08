package com.seiford.util.spark;

import com.revrobotics.CANSparkBase;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

public class SparkConfiguration {
    private final boolean inverted;
    private final IdleMode idleMode;
    private final CurrentLimitConfiguration currentLimits;
    private final StatusFrameConfiguration statusFrames;
    private final ClosedLoopConfiguration closedLoop;
    private final FeedbackConfiguration feedback;
    private final FollowingConfiguration following;

    public SparkConfiguration(boolean inverted, IdleMode idleMode, CurrentLimitConfiguration currentLimits,
            StatusFrameConfiguration statusFrames) {
        this.inverted = inverted;
        this.idleMode = idleMode;
        this.currentLimits = currentLimits;
        this.statusFrames = statusFrames;
        this.closedLoop = null;
        this.feedback = null;
        this.following = null;
    }

    public SparkConfiguration(boolean inverted, IdleMode idleMode, CurrentLimitConfiguration currentLimits,
            StatusFrameConfiguration statusFrames, ClosedLoopConfiguration closedLoop, FeedbackConfiguration feedback) {
        this.inverted = inverted;
        this.idleMode = idleMode;
        this.currentLimits = currentLimits;
        this.statusFrames = statusFrames;
        this.closedLoop = closedLoop;
        this.feedback = feedback;
        this.following = null;
    }

    public SparkConfiguration(boolean inverted, IdleMode idleMode, CurrentLimitConfiguration currentLimits,
            StatusFrameConfiguration statusFrames, FollowingConfiguration following) {
        this.inverted = inverted;
        this.idleMode = idleMode;
        this.currentLimits = currentLimits;
        this.statusFrames = statusFrames;
        this.closedLoop = null;
        this.feedback = null;
        this.following = following;
    }

    public void apply(CANSparkBase motor) {
        Util.configure(motor::restoreFactoryDefaults, false, "restoreFactoryDefaults");
        Util.configureAndVerify(motor::setInverted, motor::getInverted, inverted, "inverted");
        Util.configureCheckAndVerify(motor::setIdleMode, motor::getIdleMode, idleMode, "idleMode");
        currentLimits.apply(motor);
        statusFrames.apply(motor);
        if (closedLoop != null && feedback != null) {
            SparkPIDController pid = closedLoop.apply(motor);
            MotorFeedbackSensor sensor = feedback.apply(motor);
            pid.setFeedbackDevice(sensor);
        } else if (following != null) {
            following.apply(motor);
        }
        Util.burnFlash(motor);
    }

    public void apply(CANSparkBase... motors) {
        for (CANSparkBase motor : motors) {
            apply(motor);
        }
    }
}

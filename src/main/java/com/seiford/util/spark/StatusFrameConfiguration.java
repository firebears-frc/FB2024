package com.seiford.util.spark;

import java.util.Map;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public interface StatusFrameConfiguration {
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
    public void apply(CANSparkBase motor);

    public static StatusFrameConfiguration defaultStatusFrameConfiguration() {
        return create(Map.of(
                PeriodicFrame.kStatus0, 10,
                PeriodicFrame.kStatus1, 20,
                PeriodicFrame.kStatus2, 20,
                PeriodicFrame.kStatus3, 50,
                PeriodicFrame.kStatus4, 20,
                PeriodicFrame.kStatus5, 200,
                PeriodicFrame.kStatus6, 200));
    }

    public static StatusFrameConfiguration normal() {
        return create(Map.of(
                PeriodicFrame.kStatus0, 20,
                PeriodicFrame.kStatus1, 20,
                PeriodicFrame.kStatus2, 20,
                PeriodicFrame.kStatus3, 1000,
                PeriodicFrame.kStatus4, 1000,
                PeriodicFrame.kStatus5, 1000,
                PeriodicFrame.kStatus6, 1000));
    }

    public static StatusFrameConfiguration leader() {
        return create(Map.of(
                PeriodicFrame.kStatus0, 1,
                PeriodicFrame.kStatus1, 20,
                PeriodicFrame.kStatus2, 20,
                PeriodicFrame.kStatus3, 1000,
                PeriodicFrame.kStatus4, 1000,
                PeriodicFrame.kStatus5, 1000,
                PeriodicFrame.kStatus6, 1000));
    }

    public static StatusFrameConfiguration absoluteEncoder() {
        return create(Map.of(
                PeriodicFrame.kStatus0, 20,
                PeriodicFrame.kStatus1, 20,
                PeriodicFrame.kStatus2, 20,
                PeriodicFrame.kStatus3, 1000,
                PeriodicFrame.kStatus4, 1000,
                PeriodicFrame.kStatus5, 20,
                PeriodicFrame.kStatus6, 1000));
    }

    public static StatusFrameConfiguration absoluteEncoderLeader() {
        return create(Map.of(
                PeriodicFrame.kStatus0, 1,
                PeriodicFrame.kStatus1, 20,
                PeriodicFrame.kStatus2, 20,
                PeriodicFrame.kStatus3, 1000,
                PeriodicFrame.kStatus4, 1000,
                PeriodicFrame.kStatus5, 20,
                PeriodicFrame.kStatus6, 1000));
    }

    private static StatusFrameConfiguration create(Map<PeriodicFrame, Integer> periods) {
        return new StatusFrameConfiguration() {
            @Override
            public void apply(CANSparkBase motor) {
                for (Map.Entry<PeriodicFrame, Integer> entry : periods.entrySet()) {
                    Util.configure(period -> motor.setPeriodicFramePeriod(entry.getKey(), entry.getValue()),
                            entry.getValue(), entry.getKey().name());
                }
            }
        };
    }
}

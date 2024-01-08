package com.seiford.util.spark;

import com.revrobotics.CANSparkBase;

public interface CurrentLimitConfiguration {
    public void apply(CANSparkBase motor);

    public static CurrentLimitConfiguration simple(int smartLimit, double secondaryLimit) {
        return new CurrentLimitConfiguration() {
            @Override
            public void apply(CANSparkBase motor) {
                Util.configure(motor::setSmartCurrentLimit, smartLimit, "smartCurrentLimit");
                Util.configure(motor::setSecondaryCurrentLimit, secondaryLimit, "secondaryCurrentLimit");
            }
        };
    }

    public static CurrentLimitConfiguration complex(int stallLimit, int freeLimit, int rpmCutoff,
            double secondaryLimit) {
        return new CurrentLimitConfiguration() {
            @Override
            public void apply(CANSparkBase motor) {
                Util.configure(ignored -> motor.setSmartCurrentLimit(stallLimit, freeLimit, rpmCutoff), this,
                        "smartCurrentLimit");
                Util.configure(motor::setSecondaryCurrentLimit, secondaryLimit, "secondaryCurrentLimit");
            }
        };
    }

    public static CurrentLimitConfiguration neo() {
        return complex(40, 20, 10, 60.0);
    }

    public static CurrentLimitConfiguration neo550() {
        return complex(20, 10, 10, 30.0);
    }
}

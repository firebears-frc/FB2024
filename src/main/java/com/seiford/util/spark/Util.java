package com.seiford.util.spark;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Util {
    private static class Constants {
        public static final int MAXIMUM_RETRIES = 5;
        public static final double PRE_FLASH_DELAY = 0.2;
        public static final double POST_FLASH_DELAY = 0.05;
    }

    static <T> void configure(Function<T, REVLibError> setter, T setting, String name) {
        for (int i = 0; i < Constants.MAXIMUM_RETRIES; i++) {
            // Apply setting and check if the library returned OK status
            if (setter.apply(setting) == REVLibError.kOk) {
                return; // If it applied ok, exit
            }
            // Otherwise, try again
        }
        // We've hit max retries
        DriverStation.reportWarning("Failed to set parameter '" + name + "'", true);
    }

    static <T> void configureAndVerify(Consumer<T> setter, Supplier<T> getter, T setting, String name) {
        for (int i = 0; i < Constants.MAXIMUM_RETRIES; i++) {
            // Apply setting
            setter.accept(setting);
            // Check if the setting stuck
            if (getter.get() == setting)
                return; // If it stuck, exit
            // Otherwise, try again
        }
        // We've hit max retries
        DriverStation.reportWarning("Failed to set parameter '" + name + "'", true);
    }

    static <T> void configureCheckAndVerify(Function<T, REVLibError> setter, Supplier<T> getter, T setting, String name) {
        for (int i = 0; i < Constants.MAXIMUM_RETRIES; i++) {
            // Apply setting and check if the library returned OK status
            if (setter.apply(setting) == REVLibError.kOk) {
                // Check if the setting stuck
                if (getter.get() == setting) {
                    return; // If it applied ok and stuck, exit
                }
            }
            // Otherwise, try again
        }
        // We've hit max retries
        DriverStation.reportWarning("Failed to set parameter '" + name + "'", true);
    }

    static void burnFlash(CANSparkBase spark) {
        Timer.delay(Constants.PRE_FLASH_DELAY);
        if (spark.burnFlash() != REVLibError.kOk) {
            DriverStation.reportWarning("Failed to burn flash on " + spark.getDeviceId(), true);
            return;
        }
        Timer.delay(Constants.POST_FLASH_DELAY);
    }
}

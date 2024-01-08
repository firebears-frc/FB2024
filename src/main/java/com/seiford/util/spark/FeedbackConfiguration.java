package com.seiford.util.spark;

import java.util.function.Function;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public interface FeedbackConfiguration {
    public MotorFeedbackSensor apply(CANSparkBase motor);

    public static FeedbackConfiguration absoluteEncoder(boolean inverted, double conversionFactor) {
        return absoluteEncoder(inverted, conversionFactor, null, null);
    }

    public static FeedbackConfiguration absoluteEncoder(boolean inverted, double conversionFactor, Double zeroOffset,
            Integer averageDepth) {
        return new FeedbackConfiguration() {
            @Override
            public MotorFeedbackSensor apply(CANSparkBase motor) {
                AbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
                Util.configureCheckAndVerify(encoder::setInverted, encoder::getInverted, inverted, "inverted");
                Util.configureCheckAndVerify(encoder::setPositionConversionFactor, encoder::getPositionConversionFactor,
                        conversionFactor, "positionFactor");
                Util.configureCheckAndVerify(encoder::setVelocityConversionFactor, encoder::getVelocityConversionFactor,
                        conversionFactor / 60.0, "velocityFactor");
                if (zeroOffset != null)
                    Util.configureCheckAndVerify(encoder::setZeroOffset, encoder::getZeroOffset, zeroOffset,
                            "zeroOffset");
                if (averageDepth != null)
                    Util.configureCheckAndVerify(encoder::setAverageDepth, encoder::getAverageDepth, averageDepth,
                            "averageDepth");
                return encoder;
            }
        };
    }

    public static FeedbackConfiguration builtInEncoder(boolean inverted, double conversionFactor) {
        return builtInEncoder(inverted, conversionFactor, null, null);
    }

    public static FeedbackConfiguration builtInEncoder(boolean inverted, double conversionFactor, Integer averageDepth,
            Integer measurementPeriod) {
        return relativeEncoder(motor -> motor.getEncoder(), inverted, conversionFactor, averageDepth,
                measurementPeriod);
    }

    public static FeedbackConfiguration maxAlternativeEncoder(int countsPerRev, boolean inverted,
            double conversionFactor) {
        return maxAlternativeEncoder(countsPerRev, inverted, conversionFactor, null, null);
    }

    public static FeedbackConfiguration maxAlternativeEncoder(int countsPerRev, boolean inverted,
            double conversionFactor, Integer averageDepth, Integer measurementPeriod) {
        return relativeEncoder(motor -> {
            if (!(motor instanceof CANSparkMax))
                throw new IllegalArgumentException("Alternative encoder mode is only available on a Spark MAX");
            return ((CANSparkMax) motor).getAlternateEncoder(countsPerRev);
        }, inverted, conversionFactor, averageDepth, measurementPeriod);
    }

    public static FeedbackConfiguration flexExternalEncoder(int countsPerRev, boolean inverted,
            double conversionFactor) {
        return flexExternalEncoder(countsPerRev, inverted, conversionFactor, null, null);
    }

    public static FeedbackConfiguration flexExternalEncoder(int countsPerRev, boolean inverted, double conversionFactor,
            Integer averageDepth, Integer measurementPeriod) {
        return relativeEncoder(motor -> {
            if (!(motor instanceof CANSparkFlex))
                throw new IllegalArgumentException("External encoder mode is only available on a Spark FLEX");
            return ((CANSparkFlex) motor).getExternalEncoder(countsPerRev);
        }, inverted, conversionFactor, averageDepth, measurementPeriod);
    }

    private static FeedbackConfiguration relativeEncoder(Function<CANSparkBase, RelativeEncoder> sensorFunction,
            boolean inverted, double conversionFactor, Integer averageDepth, Integer measurementPeriod) {
        return new FeedbackConfiguration() {
            @Override
            public MotorFeedbackSensor apply(CANSparkBase motor) {
                RelativeEncoder encoder = sensorFunction.apply(motor);
                Util.configureCheckAndVerify(encoder::setInverted, encoder::getInverted, inverted, "inverted");
                Util.configureCheckAndVerify(encoder::setPositionConversionFactor, encoder::getPositionConversionFactor,
                        conversionFactor, "positionFactor");
                Util.configureCheckAndVerify(encoder::setVelocityConversionFactor, encoder::getVelocityConversionFactor,
                        conversionFactor / 60.0, "velocityFactor");
                if (averageDepth != null)
                    Util.configureCheckAndVerify(encoder::setAverageDepth, encoder::getAverageDepth, averageDepth,
                            "averageDepth");
                if (measurementPeriod != null)
                    Util.configureCheckAndVerify(encoder::setMeasurementPeriod, encoder::getMeasurementPeriod,
                            measurementPeriod, "measurementPeriod");
                return encoder;
            }
        };
    }
}

package com.seiford.util.spark;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

public class ClosedLoopConfiguration {
    private final double p, i, d, ff;
    private final double minOutput, maxOutput;
    private final Double wrappingMin, wrappingMax;
    private final Double dFilter, iZone, maxIAccum;

    public static ClosedLoopConfiguration simple(double p, double i, double d, double ff) {
        return new ClosedLoopConfiguration(p, i, d, ff, -1.0, 1.0, null, null, null, null, null);
    }

    public static ClosedLoopConfiguration wrapping(double p, double i, double d, double ff, double wrappingMin,
            double wrappingMax) {
        return new ClosedLoopConfiguration(p, i, d, ff, -1.0, 1.0, wrappingMin, wrappingMax, null, null, null);
    }

    public static ClosedLoopConfiguration outputConstraints(double p, double i, double d, double ff, double minOutput,
            double maxOutput) {
        return new ClosedLoopConfiguration(p, i, d, ff, minOutput, maxOutput, null, null, null, null, null);
    }

    public ClosedLoopConfiguration(double p, double i, double d, double ff, double minOutput, double maxOutput,
            Double wrappingMin, Double wrappingMax, Double dFilter, Double iZone, Double maxIAccum) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.ff = ff;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.wrappingMin = wrappingMin;
        this.wrappingMax = wrappingMax;
        this.dFilter = dFilter;
        this.iZone = iZone;
        this.maxIAccum = maxIAccum;
    }

    public SparkPIDController apply(CANSparkBase motor) {
        SparkPIDController pid = motor.getPIDController();
        Util.configureCheckAndVerify(pid::setP, pid::getP, p, "p");
        Util.configureCheckAndVerify(pid::setI, pid::getI, i, "i");
        Util.configureCheckAndVerify(pid::setD, pid::getD, d, "d");
        Util.configureCheckAndVerify(pid::setFF, pid::getFF, ff, "ff");
        Util.configureCheckAndVerify(ignored -> pid.setOutputRange(minOutput, maxOutput),
                () -> pid.getOutputMin() == minOutput && pid.getOutputMax() == maxOutput, true, "outputRange");
        if (wrappingMin != null && wrappingMax != null) {
            Util.configureCheckAndVerify(pid::setPositionPIDWrappingEnabled, pid::getPositionPIDWrappingEnabled, true,
                    "wrappingEnabled");
            Util.configureCheckAndVerify(pid::setPositionPIDWrappingMinInput, pid::getPositionPIDWrappingMinInput,
                    minOutput, "wrappingMinInput");
            Util.configureCheckAndVerify(pid::setPositionPIDWrappingMaxInput, pid::getPositionPIDWrappingMaxInput,
                    maxOutput, "wrappingMaxInput");
        }
        if (dFilter != null)
            Util.configureCheckAndVerify(pid::setDFilter, () -> pid.getDFilter(0), dFilter, "dFilter");
        if (iZone != null)
            Util.configureCheckAndVerify(pid::setIZone, pid::getIZone, iZone, "iZone");
        if (maxIAccum != null)
            Util.configureCheckAndVerify(setting -> pid.setIMaxAccum(setting, 0), () -> pid.getIMaxAccum(0), maxIAccum,
                    "maxIAccum");
        return pid;
    }
}

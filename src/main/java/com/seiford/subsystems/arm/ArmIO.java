package com.seiford.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public Rotation2d position = new Rotation2d();
    public double[] appliedVolts = new double[] {};
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {
  }

  /** Run closed loop to the specified velocity. */
  public default void setPosition(Rotation2d angle, double ffVolts) {
  }

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {
  }
}

package com.seiford.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {
  }

  /** Stop in open loop. */
  public default void stop() {
  }
}

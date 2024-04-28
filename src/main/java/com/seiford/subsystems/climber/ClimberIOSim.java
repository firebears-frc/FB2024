package com.seiford.subsystems.climber;

import org.littletonrobotics.junction.Logger;

public class ClimberIOSim implements ClimberIO {
  @Override
  public void updateInputs(ClimberIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {
    Logger.recordOutput("Climber/Voltage", volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}

package com.seiford.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberIOSparkMax implements ClimberIO {
  private final CANSparkMax right = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax left = new CANSparkMax(15, MotorType.kBrushless);

  public ClimberIOSparkMax() {
    right.restoreFactoryDefaults();
    left.restoreFactoryDefaults();

    right.setCANTimeout(250);
    left.setCANTimeout(250);

    right.setInverted(false);
    left.setInverted(false);

    right.enableVoltageCompensation(12.0);
    right.setSmartCurrentLimit(40, 20);
    right.setSecondaryCurrentLimit(45.0);
    left.enableVoltageCompensation(12.0);
    left.setSmartCurrentLimit(40, 20);
    left.setSecondaryCurrentLimit(45.0);

    right.burnFlash();
    left.burnFlash();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
  }

  @Override
  public void setVoltage(double volts) {
    right.setVoltage(volts);
    left.setVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}

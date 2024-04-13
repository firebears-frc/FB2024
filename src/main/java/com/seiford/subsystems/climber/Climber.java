package com.seiford.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public final class Constants {
    public static final double CLIMB_VOLTAGE = 12.0;
    public static final double REVERSE_VOLTAGE = -9.0;
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  /** Run open loop at the specified voltage. */
  private void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Returns a command to run the climber up. */
  public Command climb() {
    return runOnce(() -> setVoltage(Constants.CLIMB_VOLTAGE));
  }

  /** Returns a command to run the climber down. */
  public Command reverse() {
    return runOnce(() -> setVoltage(Constants.REVERSE_VOLTAGE));
  }

  /** Returns a command to stop the climber. */
  public Command stop() {
    return runOnce(() -> io.stop());
  }
}

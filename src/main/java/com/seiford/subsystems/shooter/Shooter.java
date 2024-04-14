// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.seiford.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.seiford.subsystems.shooter.ShooterIOInputsAutoLogged;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.seiford.Configuration;

public class Shooter extends SubsystemBase {
  public static final class Constants {
    public static final double GEAR_RATIO = 1.5;
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final LoggedDashboardNumber speakerInput = new LoggedDashboardNumber("Shooter Speaker Speed", 2400.0);
  private final LoggedDashboardNumber ampInput = new LoggedDashboardNumber("Shooter Amp Speed", 625.0);
  private final LoggedDashboardNumber ejectInput = new LoggedDashboardNumber("Shooter Eject Speed", -625.0);
  private final SimpleMotorFeedforward ffModel;
  private final Debouncer debouncer = new Debouncer(0.2);
  private final SysIdRoutine sysId;
  @AutoLogOutput(key = "Shooter/Setpoint")
  private double setpoint = 0.0;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Configuration.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  @AutoLogOutput(key = "Shooter/Speed")
  private double getSpeed() {
    double result = 0;
    for (double value : inputs.velocitiesRadPerSec) {
      result += value;
    }
    result /= inputs.velocitiesRadPerSec.length;
    return Units.radiansPerSecondToRotationsPerMinute(result);
  }

  @AutoLogOutput(key = "Shooter/Error")
  private double getError() {
    return getSpeed() - setpoint;
  }

  @AutoLogOutput(key = "Shooter/AtSpeed")
  private boolean atSpeed() {
    return Math.abs(getError()) < 100.0;
  }

  @AutoLogOutput(key = "Shooter/OnTarget")
  private boolean onTarget() {
    return debouncer.calculate(atSpeed());
  }

  /** Run open loop at the specified voltage. */
  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    setpoint = velocityRPM;
  }

  /** Stops the shooter. */
  private void stop() {
    io.stop();
  }

  /** Returns a command to run the shooter at a set speed. */
  private Command speedCommand(double velocityRPM) {
    return Commands.sequence(
        runOnce(() -> runVelocity(velocityRPM)),
        Commands.waitSeconds(0.25),
        run(() -> {
        }).until(this::onTarget));
  }

  /** Returns a command to run the shooter at speaker speed. */
  public Command speaker() {
    return speedCommand(speakerInput.get());
  }

  /** Returns a command to run the shooter at amp speed. */
  public Command amp() {
    return speedCommand(ampInput.get());
  }

  /** Returns a command to run the shooter at eject speed. */
  public Command eject() {
    return speedCommand(ejectInput.get());
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}

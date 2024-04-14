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

package com.seiford.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.seiford.subsystems.intake.IntakeIOInputsAutoLogged;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.seiford.Configuration;

public class Intake extends SubsystemBase {
  public final class Constants {
    public static final double GEAR_RATIO = 3.0;
  }

  private final LoggedDashboardNumber intakeSpeedInput = new LoggedDashboardNumber("Intake/Intake Speed", 2000.0);
  private final LoggedDashboardNumber shootSpeedInput = new LoggedDashboardNumber("Intake/Shoot Speed", 2625.0);
  private final LoggedDashboardNumber ejectSpeedInput = new LoggedDashboardNumber("Intake/Eject Speed", -1375.0);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  @AutoLogOutput(key = "Intake/HasNote")
  private boolean hasNote;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
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
            (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Stop the intake if we have a note
    if (inputs.beamBrake && !hasNote) {
      stopIntake();
      hasNote = true;
    } else if (!inputs.beamBrake) {
      hasNote = false;
    }
  }

  /** Run open loop at the specified voltage. */
  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log Intake setpoint
    Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
  }

  /** Stops the intake. */
  private void stopIntake() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  private double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns a command to run the intake at intake speed. */
  public Command intake() {
    return runOnce(() -> runVelocity(intakeSpeedInput.get()));
  }

  /** Returns a command to run the intake at shoot speed. */
  public Command shoot() {
    return runOnce(() -> runVelocity(shootSpeedInput.get()));
  }

  /** Returns a command to run the intake at eject speed. */
  public Command eject() {
    return runOnce(() -> runVelocity(ejectSpeedInput.get()));
  }

  /** Returns a command to stop the intake. */
  public Command stop() {
    return runOnce(this::stopIntake);
  }

  /** Returns a command to run the intake at intake speed until it has a note. */
  public Command autoIntake() {
    return Commands.sequence(
        intake(),
        Commands.waitSeconds(0.05),
        run(() -> {
        }).until(() -> hasNote));
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

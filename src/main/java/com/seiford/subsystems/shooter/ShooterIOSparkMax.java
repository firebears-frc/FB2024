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

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of
 * "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ShooterIOSparkMax implements ShooterIO {
  private final CANSparkMax topMotor = new CANSparkMax(10, MotorType.kBrushless);
  private final RelativeEncoder topEncoder = topMotor.getEncoder();
  private final SparkPIDController topPID = topMotor.getPIDController();
  private final CANSparkMax bottomMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder bottomEncoder = topMotor.getEncoder();
  private final SparkPIDController bottomPID = topMotor.getPIDController();

  public ShooterIOSparkMax() {
    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    topMotor.setCANTimeout(250);
    bottomMotor.setCANTimeout(250);

    topMotor.setInverted(false);
    bottomMotor.setInverted(false);

    topMotor.enableVoltageCompensation(12.0);
    topMotor.setSmartCurrentLimit(50, 30);
    topMotor.setSecondaryCurrentLimit(60.0);
    bottomMotor.enableVoltageCompensation(12.0);
    bottomMotor.setSmartCurrentLimit(50, 30);
    bottomMotor.setSecondaryCurrentLimit(60.0);

    topMotor.burnFlash();
    bottomMotor.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocitiesRadPerSec = new double[] {
        Units.rotationsPerMinuteToRadiansPerSecond(topEncoder.getVelocity() / Shooter.Constants.GEAR_RATIO),
        Units.rotationsPerMinuteToRadiansPerSecond(bottomEncoder.getVelocity() / Shooter.Constants.GEAR_RATIO)
    };
    inputs.appliedVolts = new double[] {
        topMotor.getAppliedOutput() * topMotor.getBusVoltage(),
        bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage()
    };
    inputs.currentAmps = new double[] { topMotor.getOutputCurrent(), bottomMotor.getOutputCurrent() };
  }

  @Override
  public void setVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    topPID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * Shooter.Constants.GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
    bottomPID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * Shooter.Constants.GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    topPID.setP(kP, 0);
    topPID.setI(kI, 0);
    topPID.setD(kD, 0);
    topPID.setFF(0, 0);
    bottomPID.setP(kP, 0);
    bottomPID.setI(kI, 0);
    bottomPID.setD(kD, 0);
    bottomPID.setFF(0, 0);
  }
}

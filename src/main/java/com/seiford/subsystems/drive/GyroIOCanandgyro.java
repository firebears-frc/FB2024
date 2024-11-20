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

package com.seiford.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

/** IO implementation for Canandgyro */
public class GyroIOCanandgyro implements GyroIO {
  public final Canandgyro gyro = new Canandgyro(0);
  public static final boolean INVERTED = true;

  public GyroIOCanandgyro() {
    gyro.setYaw(0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawPosition = INVERTED ? gyro.getRotation2d().unaryMinus() : gyro.getRotation2d();
  }
}

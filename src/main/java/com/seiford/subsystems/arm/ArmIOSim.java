package com.seiford.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim sim = new SingleJointedArmSim(
      DCMotor.getNEO(2),
      Arm.Constants.GEAR_RATIO,
      Units.lbsToKilograms(18.0) * Math.pow(Units.inchesToMeters(18.0), 2),
      Units.inchesToMeters(37.0),
      Arm.Constants.MINIMUM.getRadians(),
      Arm.Constants.MAXIMUM.getRadians(),
      true,
      Rotation2d.fromDegrees(80.0).getRadians());
  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngleRads()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.position = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.appliedVolts = new double[] { appliedVolts, appliedVolts };
    inputs.currentAmps = new double[] { sim.getCurrentDrawAmps(), sim.getCurrentDrawAmps() };
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPosition(Rotation2d angle, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(angle.getRadians());
    this.ffVolts = ffVolts;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}

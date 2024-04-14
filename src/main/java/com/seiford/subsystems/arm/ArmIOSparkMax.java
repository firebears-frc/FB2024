package com.seiford.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmIOSparkMax implements ArmIO {
  private final CANSparkMax leader = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(12, MotorType.kBrushless);
  private final SparkAbsoluteEncoder encoder = leader.getAbsoluteEncoder();
  private final SparkPIDController pid = leader.getPIDController();

  public ArmIOSparkMax() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    leader.setCANTimeout(250);
    follower.setCANTimeout(250);

    leader.setInverted(false);
    follower.follow(leader, true);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(50, 30);
    leader.setSecondaryCurrentLimit(60.0);
    follower.enableVoltageCompensation(12.0);
    follower.setSmartCurrentLimit(50, 30);
    follower.setSecondaryCurrentLimit(60.0);

    leader.burnFlash();
    follower.burnFlash();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.position = Rotation2d.fromRotations(encoder.getPosition());
    inputs.appliedVolts = new double[] {
        leader.getAppliedOutput() * leader.getBusVoltage(),
        follower.getAppliedOutput() * follower.getBusVoltage()
    };
    inputs.currentAmps = new double[] { leader.getOutputCurrent(), follower.getOutputCurrent() };
  }

  @Override
  public void setPosition(Rotation2d angle, double ffVolts) {
    pid.setReference(
        angle.getRotations(),
        ControlType.kPosition,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0.0, 0);
  }
}

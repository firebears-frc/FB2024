package com.seiford.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.seiford.Configuration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  public static final class Constants {
    public static final double GEAR_RATIO = 192.0;

    public static final Rotation2d MINIMUM = Rotation2d.fromDegrees(-5.0);
    public static final Rotation2d MAXIMUM = Rotation2d.fromDegrees(100.0);
  }
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final LoggedDashboardNumber intakeInput = new LoggedDashboardNumber("Intake Angle", 0.0);
  private final LoggedDashboardNumber ampInput = new LoggedDashboardNumber("Amp Angle", 90.0);
  private final LoggedDashboardNumber speakerInput = new LoggedDashboardNumber("Speaker Angle", 13.5);
  private final ArmFeedforward ffModel;
  @AutoLogOutput(key = "Arm/Setpoint")
  private Rotation2d setpoint;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Configuration.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new ArmFeedforward(0.1, 0.35, 3.74, 0.02);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new ArmFeedforward(0.0, 0.35, 3.74, 0.02);
        io.configurePID(25.0, 0.0, 0.0);
        break;
      default:
        ffModel = new ArmFeedforward(0.0, 0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    if (setpoint == null) {
      // On init, set the setpoint to the current position
      setpoint = inputs.position;
    }

    double ffVolts = ffModel.calculate(inputs.position.getRadians(), 0.0);
    Logger.recordOutput("Arm/ffVolts", ffVolts);
    io.setPosition(setpoint, ffVolts);
  }

  /** Run closed loop at the specified position. */
  private void runPosition(Rotation2d angle) {
    if (angle.getDegrees() > Constants.MAXIMUM.getDegrees())
      setpoint = Constants.MAXIMUM;
    else if (angle.getDegrees() < Constants.MINIMUM.getDegrees())
      setpoint = Constants.MINIMUM;
    else
      setpoint = angle;
  }

  /** Returns a command to move the arm to the specified position. */
  private Command positionCommand(Rotation2d angle) {
    return runOnce(() -> runPosition(angle)); // TODO
  }

  /** Returns a command to move the arm to intake position. */
  public Command intake() {
    return positionCommand(Rotation2d.fromDegrees(intakeInput.get()));
  }

  /** Returns a command to move the arm to speaker position. */
  public Command speaker() {
    return positionCommand(Rotation2d.fromDegrees(speakerInput.get()));
  }

  /** Returns a command to move the arm to amp position. */
  public Command amp() {
    return positionCommand(Rotation2d.fromDegrees(ampInput.get()));
  }
}

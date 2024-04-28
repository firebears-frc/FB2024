package com.seiford.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.seiford.Configuration;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Arm extends SubsystemBase {
  public static final class Constants {
    public static final double GEAR_RATIO = 192.0;

    public static final Rotation2d MINIMUM = Rotation2d.fromDegrees(-5.0);
    public static final Rotation2d MAXIMUM = Rotation2d.fromDegrees(100.0);
  }

  private static enum State {
    STARTUP,
    INTAKE,
    SPEAKER,
    AMP,
    STOW,
    SYSID
  }

  private final LoggedDashboardNumber intakeInput =
      new LoggedDashboardNumber("Arm/Intake Angle", 0.0);
  private final LoggedDashboardNumber ampInput = new LoggedDashboardNumber("Arm/Amp Angle", 90.0);
  private final LoggedDashboardNumber stowInput = new LoggedDashboardNumber("Arm/Stow Angle", 45.0);

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmFeedforward ffModel;
  private final Debouncer debouncer = new Debouncer(0.2);
  private final Supplier<Rotation2d> angleSupplier;
  private final SysIdRoutine sysId;

  @AutoLogOutput(key = "Arm/State")
  private State state = State.STARTUP;

  @AutoLogOutput(key = "Arm/Setpoint")
  private Rotation2d setpoint;

  /** Creates a new Arm. */
  public Arm(ArmIO io, Supplier<Rotation2d> angleSupplier) {
    this.io = io;
    this.angleSupplier = angleSupplier;

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

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    switch (state) {
      case STARTUP:
        // On init, set the setpoint to the current position
        if (setpoint == null) setAngle(inputs.position);
        break;
      case INTAKE:
        setAngle(Rotation2d.fromDegrees(intakeInput.get()));
        break;
      case SPEAKER:
        setAngle(angleSupplier.get());
        break;
      case AMP:
        setAngle(Rotation2d.fromDegrees(ampInput.get()));
        break;
      case STOW:
        setAngle(Rotation2d.fromDegrees(stowInput.get()));
        break;
      case SYSID:
        // TODO
        break;
    }

    double ffVolts = ffModel.calculate(inputs.position.getRadians(), 0.0);
    Logger.recordOutput("Arm/ffVolts", ffVolts);
    io.setPosition(setpoint, ffVolts);
  }

  @AutoLogOutput(key = "Arm/Error")
  private Rotation2d getError() {
    return inputs.position.minus(setpoint);
  }

  @AutoLogOutput(key = "Arm/AtPosition")
  private boolean atSpeed() {
    return Math.abs(getError().getDegrees()) < 1.0;
  }

  @AutoLogOutput(key = "Arm/OnTarget")
  private boolean onTarget() {
    return debouncer.calculate(atSpeed());
  }

  /** Run open loop at the specified voltage. */
  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  private void setAngle(Rotation2d angle) {
    if (angle.getDegrees() > Constants.MAXIMUM.getDegrees()) setpoint = Constants.MAXIMUM;
    else if (angle.getDegrees() < Constants.MINIMUM.getDegrees()) setpoint = Constants.MINIMUM;
    else setpoint = angle;
  }

  /** Returns a command to move the arm to the specified state. */
  private Command stateCommand(State state) {
    return Commands.sequence(
        runOnce(() -> this.state = state),
        Commands.waitSeconds(0.25),
        run(() -> {}).until(this::onTarget));
  }

  /** Returns a command to move the arm to intake state. */
  public Command intake() {
    return stateCommand(State.INTAKE);
  }

  /** Returns a command to move the arm to speaker state. */
  public Command speaker() {
    return stateCommand(State.SPEAKER);
  }

  /** Returns a command to move the arm to amp state. */
  public Command amp() {
    return stateCommand(State.AMP);
  }

  /** Returns a command to move the arm to stow state. */
  public Command stow() {
    return stateCommand(State.STOW);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(() -> state = State.SYSID), sysId.quasistatic(direction), amp());
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(runOnce(() -> state = State.SYSID), sysId.dynamic(direction), amp());
  }
}

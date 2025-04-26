package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class DownBeat extends SubsystemBase {
  private SparkFlex downBeatMotor;
  private SparkClosedLoopController pid;
  private DigitalInput sensor;
  private double setPoint = 0;

  @AutoLogOutput(key = "downBeat/hasNote")
  private boolean hasNote = false;

  private final LoggedNetworkNumber shootSpeed =
      new LoggedNetworkNumber("downBeat/shootSpeed", 2700);

  public DownBeat() {
    downBeatMotor = new SparkFlex(9, MotorType.kBrushless);
    pid = downBeatMotor.getClosedLoopController();

    var downBeatMotorConfig = new SparkFlexConfig();
    downBeatMotorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(80, 20)
        .secondaryCurrentLimit(100);
    downBeatMotorConfig.closedLoop.pidf(0.0001, 0.0, 0.0, 0.00022).iZone(100);
    SparkUtil.tryUntilOk(
        downBeatMotor,
        5,
        () ->
            downBeatMotor.configure(
                downBeatMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    // downBeatMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    // downBeatMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    // downBeatMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    // sensor
    sensor = new DigitalInput(0);
  }

  @AutoLogOutput(key = "downBeat/beamBreak")
  private boolean beamBreak() {
    return sensor.get();
  }

  @AutoLogOutput(key = "downBeat/error")
  private double getError() {
    return setPoint - downBeatMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "downBeat/atSpeed")
  private boolean atSpeed() {
    if ((getError() < 100) && (getError() > -100)) {
      return true;
    }
    return false;
  }

  public Command intakeNote() {
    return runOnce(
        () -> {
          setPoint = 2100;
        });
  }

  public Command shootNote() {
    return runOnce(
        () -> {
          setPoint = shootSpeed.get();
        });
  }

  public Command dischargeNote() {
    return runOnce(
        () -> {
          setPoint = -2100;
        });
  }

  public Command pauseDownBeat() {
    return runOnce(
        () -> {
          setPoint = 0;
        });
  }

  public Command autoIntake(double timeOut) {
    return Commands.sequence(
        runOnce(() -> setPoint = shootSpeed.get()),
        run(() -> {}).until(() -> hasNote).withTimeout(timeOut));
  }

  @Override
  public void periodic() {
    if (beamBreak() && !hasNote) {
      setPoint = 0;
      hasNote = true;
    } else if (!beamBreak()) {
      hasNote = false;
    }
    pid.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("downBeat/Output", downBeatMotor.getAppliedOutput());
    Logger.recordOutput("downBeat/speed", downBeatMotor.getEncoder().getVelocity());
    Logger.recordOutput("downBeat/setPoint", setPoint);
  }
}

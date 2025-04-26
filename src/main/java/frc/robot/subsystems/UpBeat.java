package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class UpBeat extends SubsystemBase {
  private SparkMax topMotor;
  private SparkClosedLoopController topPid;
  private SparkMax bottomMotor;
  private SparkClosedLoopController bottomPid;

  @AutoLogOutput(key = "upBeat/setPoint")
  private double setPoint = 0;

  private final LoggedNetworkNumber shootSpeed = new LoggedNetworkNumber("upBeat/shootSpeed", 3600);

  private Debouncer debounce = new Debouncer(0.2);

  public UpBeat() {
    topMotor = new SparkMax(10, MotorType.kBrushless);
    bottomMotor = new SparkMax(11, MotorType.kBrushless);
    topPid = topMotor.getClosedLoopController();
    bottomPid = bottomMotor.getClosedLoopController();

    var topMotorConfig = new SparkMaxConfig();
    topMotorConfig
        .smartCurrentLimit(50, 50)
        .secondaryCurrentLimit(60)
        .inverted(false)
        .idleMode(IdleMode.kCoast);
    topMotorConfig
        .closedLoop
        .pidf(0.0003, 0.0000001, 0.0, 0.0001875)
        .iZone(100)
        .outputRange(0.0, 1.0);
    SparkUtil.tryUntilOk(
        topMotor,
        5,
        () ->
            topMotor.configure(
                topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var bottomMotorConfig = new SparkMaxConfig();
    bottomMotorConfig
        .smartCurrentLimit(50, 50)
        .secondaryCurrentLimit(60)
        .inverted(false)
        .idleMode(IdleMode.kCoast);
    bottomMotorConfig
        .closedLoop
        .pidf(0.0003, 0.0000001, 0.0, 0.0001875)
        .iZone(100)
        .outputRange(0.0, 1.0);
    SparkUtil.tryUntilOk(
        bottomMotor,
        5,
        () ->
            bottomMotor.configure(
                bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    // topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    // topMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    // bottomMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    // bottomMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    // bottomMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
  }

  private static final class Constants {
    private static final double stop = 0.00;
    private static final double reverse = -1000.00;
    private static final double amp = 1000.00;
  }

  @AutoLogOutput(key = "upBeat/speed")
  private double getSpeed() {
    return bottomMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "upBeat/error")
  private double getError() {
    return setPoint - getSpeed();
  }

  @AutoLogOutput(key = "upBeat/at speed")
  private boolean atSpeed() {
    return Math.abs(getError()) < 100;
  }

  @AutoLogOutput(key = "upBeat/at debouncespeed")
  private boolean debounceSpeend() {
    return debounce.calculate(atSpeed());
  }

  private Command speedCommand(Supplier<Double> speed) {
    return Commands.sequence(
        runOnce(() -> setPoint = speed.get()),
        Commands.waitSeconds(0.1),
        run(() -> {}).until(this::debounceSpeend));
  }

  public Command shootNote() {
    return startEnd(() -> setPoint = shootSpeed.get(), () -> setPoint = Constants.stop);
  }

  public Command reverseShootNote() {
    return speedCommand(() -> Constants.reverse);
  }

  public Command pauseUpBeat() {
    return speedCommand(() -> Constants.stop);
  }

  public Command ampSpeed() {
    return speedCommand(() -> Constants.amp);
  }

  public Command autoShoot() {
    return speedCommand(shootSpeed::get);
  }

  @Override
  public void periodic() {
    topPid.setReference(setPoint, ControlType.kVelocity);
    bottomPid.setReference(setPoint, ControlType.kVelocity);

    Logger.recordOutput("upBeat/topOutput", topMotor.getAppliedOutput());
    Logger.recordOutput("upBeat/bottomOutput", bottomMotor.getAppliedOutput());
    Logger.recordOutput("upBeat/topSpeed", topMotor.getEncoder().getVelocity());
    Logger.recordOutput("upBeat/bottomSpeed", bottomMotor.getEncoder().getVelocity());
  }
}

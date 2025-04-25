package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.utils.SparkUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private static int STALL_CURRENT_LIMIT_SHOULDER = 20;
  private static int FREE_CURRENT_LIMIT_SHOULDER = 20;
  private static int SECONDARY_CURRENT_LIMIT_SHOULDER = 30;
  private final SparkMax shoulderMotorRight;
  private final SparkMax shoulderMotorLeft;
  private final SparkAbsoluteEncoder shoulderEncoder;
  private final SparkClosedLoopController shoulderPID;

  @AutoLogOutput(key = "arm/setPoint")
  private Rotation2d shoulderSetpoint = new Rotation2d();

  private Debouncer debounce = new Debouncer(0.2);

  public Arm() {
    shoulderMotorRight = new SparkMax(13, MotorType.kBrushless);
    shoulderMotorLeft = new SparkMax(12, MotorType.kBrushless);
    shoulderEncoder = shoulderMotorRight.getAbsoluteEncoder();
    shoulderPID = shoulderMotorRight.getClosedLoopController();

    var shoulderMotorRightConfig = new SparkMaxConfig();
    shoulderMotorRightConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER)
        .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
    shoulderMotorRightConfig.absoluteEncoder.inverted(true).positionConversionFactor(360);
    shoulderMotorRightConfig
        .closedLoop
        .pid(ArmConstants.shoulderP, ArmConstants.shoulderI, ArmConstants.shoulderD)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 360)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    SparkUtil.tryUntilOk(
        shoulderMotorRight,
        5,
        () ->
            shoulderMotorRight.configure(
                shoulderMotorRightConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    var shoulderMotorLeftConfig = new SparkMaxConfig();
    shoulderMotorLeftConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER)
        .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER)
        .follow(shoulderMotorRight);
    SparkUtil.tryUntilOk(
        shoulderMotorLeft,
        5,
        () ->
            shoulderMotorLeft.configure(
                shoulderMotorLeftConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    // shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    // shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    // shoulderMotorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    // shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    // shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    // shoulderMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    setShoulderSetpoint(getShoulderAngle());
  }

  private static final class Constants { // arm setpoints
    private static final Rotation2d pickUp = Rotation2d.fromDegrees(0);
    private static final Rotation2d speakerShoot = Rotation2d.fromDegrees(13.5);
    private static final Rotation2d ampShoot = Rotation2d.fromDegrees(90);
    // private static final Rotation2d stow = Rotation2d.fromDegrees(20);
    private static final Rotation2d sideShoot = Rotation2d.fromDegrees(33.75);
    private static final Rotation2d straightShot = Rotation2d.fromDegrees(14.5);
  }

  @AutoLogOutput(key = "arm/Angle")
  public Rotation2d getShoulderAngle() {
    return Rotation2d.fromDegrees(shoulderEncoder.getPosition());
  }

  public void setShoulderSetpoint(Rotation2d setpoint) {
    if (setpoint.getDegrees() < -5) {
      setpoint = Rotation2d.fromDegrees(-5);
    } else if (setpoint.getDegrees() > 100) {
      setpoint = Rotation2d.fromDegrees(100);
    }
    shoulderSetpoint = setpoint;
  }

  @AutoLogOutput(key = "arm/error")
  private Rotation2d getError() {
    return getShoulderAngle().minus(shoulderSetpoint);
  }

  public Command defaultCommand(Supplier<Double> shoulderChange) {
    return run(
        () -> {
          setShoulderSetpoint(shoulderSetpoint.minus(Rotation2d.fromDegrees(shoulderChange.get())));
        });
  }

  public Command pickUp() {
    return positionCommand(Constants.pickUp, 1);
  }

  public Command speakerShoot() {
    return positionCommand(Constants.speakerShoot, 1);
  }

  public Command ampShoot() {
    return positionCommand(Constants.ampShoot, 1);
  }

  public Command sideShoot() {
    return positionCommand(Constants.sideShoot, 1);
  }

  public Command straightShot() {
    return positionCommand(Constants.straightShot, 1);
  }

  private boolean onTarget(double tolerance) {
    boolean onTarget = Math.abs(getError().getDegrees()) < tolerance;
    Logger.recordOutput("arm/onTargt", onTarget);
    boolean debounced = debounce.calculate(onTarget);
    Logger.recordOutput("arm/at debouncespeed", debounced);
    return debounced;
  }

  private Command positionCommand(Rotation2d position, double tolerance) {
    return Commands.sequence(
        runOnce(() -> setShoulderSetpoint(position)),
        Commands.waitSeconds(0.1),
        run(() -> {}).until(() -> onTarget(tolerance)));
  }

  public Command groundSlam() {
    return Commands.sequence(
        runOnce(() -> setShoulderSetpoint(Constants.pickUp)),
        Commands.waitSeconds(0.1),
        run(() -> {}).until(() -> getShoulderAngle().getDegrees() < 5));
  }

  @Override
  public void periodic() {
    double feedForward = Math.cos(getShoulderAngle().getRadians()) * ArmConstants.shoulderG;
    shoulderPID.setReference(shoulderSetpoint.getDegrees(), ControlType.kPosition);

    Logger.recordOutput("arm/MotorLeft", shoulderMotorLeft.getAppliedOutput());
    Logger.recordOutput("arm/MotorRight", shoulderMotorRight.getAppliedOutput());
    Logger.recordOutput("arm/MotorLeftCurrent", shoulderMotorLeft.getOutputCurrent());
    Logger.recordOutput("arm/MotorRightCurrent", shoulderMotorRight.getOutputCurrent());
    Logger.recordOutput("arm/setPointDegrees", shoulderSetpoint.getDegrees());
    Logger.recordOutput("arm/FeedForward", feedForward);
  }
}

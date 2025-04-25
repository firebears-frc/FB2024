package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.utils.SparkUtil;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    var drivingConfig = new SparkMaxConfig();
    drivingConfig
        .idleMode(ModuleConstants.kDrivingMotorIdleMode)
        .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    drivingConfig
        .encoder
        .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
        .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)
        .quadratureAverageDepth(2)
        .quadratureMeasurementPeriod(16);
    drivingConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            ModuleConstants.kDrivingP,
            ModuleConstants.kDrivingI,
            ModuleConstants.kDrivingD,
            ModuleConstants.kDrivingFF)
        .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);
    var turningConfig = new SparkMaxConfig();
    turningConfig
        .idleMode(ModuleConstants.kTurningMotorIdleMode)
        .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    turningConfig
        .absoluteEncoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
        .inverted(ModuleConstants.kTurningEncoderInverted);
    turningConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(
            ModuleConstants.kTurningEncoderPositionPIDMinInput,
            ModuleConstants.kTurningEncoderPositionPIDMaxInput)
        .pidf(
            ModuleConstants.kTurningP,
            ModuleConstants.kTurningI,
            ModuleConstants.kTurningD,
            ModuleConstants.kTurningFF)
        .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

    // m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    // m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    // m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    // m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    // m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    // m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    SparkUtil.tryUntilOk(
        m_drivingSparkMax,
        5,
        () ->
            m_drivingSparkMax.configure(
                drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        m_turningSparkMax,
        5,
        () ->
            m_turningSparkMax.configure(
                turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = new SwerveModuleState();
    optimizedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(
        optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(
        optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }
}

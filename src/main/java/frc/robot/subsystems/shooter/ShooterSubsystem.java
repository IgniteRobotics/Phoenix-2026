// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.preferences.DoublePreference;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_shooterMotor;
  private final SparkMax m_shooterIndexMotor;
  private final TalonFX m_shooterPositionMotor;
  private final CANcoder m_shooterPositionCancoder;
  private final Servo m_RightServo;
  private final Servo m_LeftServo;

  private final RelativeEncoder m_shooterEncoder;
  private final RelativeEncoder m_shooterIndexEncoder;
  private final SparkClosedLoopController m_RollerPidController;

  // TODO change these name
  private Slot0Configs positionSlot0Configs = new Slot0Configs();

  // shooterRoller preferences
  private DoublePreference shooterkPPreference =
      new DoublePreference("shooter/RPMkP", ShooterConstants.ROLLER_kP);
  private DoublePreference shooterkIPreference =
      new DoublePreference("shooter/RPMkI", ShooterConstants.ROLLER_kI);
  private DoublePreference shooterkDPreference =
      new DoublePreference("shooter/RPMkD", ShooterConstants.ROLLER_kD);
  private DoublePreference shooterkFPreference =
      new DoublePreference("shooter/RPMkF", ShooterConstants.ROLLER_kF);
  private SoftwareLimitSwitchConfigs m_positionSoftLimitConfig = new SoftwareLimitSwitchConfigs();
  private MotionMagicConfigs m_positionMotionMagicConfigs = new MotionMagicConfigs();
  private MotorOutputConfigs m_positionMotorConfig = new MotorOutputConfigs();
  private TalonFXConfiguration m_fxCfg = new TalonFXConfiguration();

  public MotionMagicVoltage shooterPosition = new MotionMagicVoltage(0);

  private DigitalInput m_indexerBeamBreak = new DigitalInput(0);

  /*********************  Telemetry Variables *********************/

  @Logged private double temp;

  @Logged private double current;

  @Logged private double velocity;

  @Logged private double targetVelocity;

  @Logged private double tempIndex;

  @Logged private double currentIndex;

  @Logged private double velocityIndex;

  @Logged private double targetPosition = 0;

  @Logged public double robotVelocity;

  @Logged public Pose2d robotPose2d;

  @Logged public double armPosition;

  @Logged public double armVelocity;

  @Logged public double armCancoderPosition;

  @Logged public double armCancoderVelocity;

  @Logged public double armPower;

  @Logged public double armVoltage;

  @Logged public double armTemp;

  @Logged public double armCurrent;

  private String armNeutralMode;

  private boolean armCurrentFault;

  private boolean armRevLimiFault;

  @Logged private boolean Ready = true;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    m_shooterMotor = new SparkMax(ShooterConstants.SHOOTER_MOTOR_LEADERCanId, MotorType.kBrushless);
    m_shooterIndexMotor = new SparkMax(ShooterConstants.SHOOTER_INDEX_MOTOR, MotorType.kBrushless);
    m_shooterPositionMotor = new TalonFX(ShooterConstants.SHOOTER_POSITION_MOTOR);
    m_shooterPositionCancoder = new CANcoder(ShooterConstants.SHOOTER_POSITION_CANCODER);

    m_shooterEncoder = m_shooterMotor.getEncoder();
    m_shooterIndexEncoder = m_shooterIndexMotor.getEncoder();

    m_RollerPidController = m_shooterMotor.getClosedLoopController();

    m_RightServo = new Servo(ShooterConstants.RIGHT_SERVO_PORT);
    m_LeftServo = new Servo(ShooterConstants.LEFT_SERVO_PORT);

    this.configureShooterMotor(m_shooterMotor, m_shooterEncoder, m_RollerPidController);
    this.configureIndexMotor(m_shooterIndexMotor);
    this.configureCancoder(m_shooterPositionCancoder);
    this.configurePositionMotor(
        m_shooterPositionMotor,
        m_fxCfg,
        m_positionMotorConfig,
        positionSlot0Configs,
        m_positionSoftLimitConfig,
        m_positionMotionMagicConfigs);
  }

  private void configureShooterMotor(
      SparkMax motor, RelativeEncoder encoder, SparkClosedLoopController pidController) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.encoder.velocityConversionFactor(1);
    config.inverted(false);
    config.idleMode(IdleMode.kCoast);
    // TODO: Make actual constants
    config.smartCurrentLimit(40);
    config.closedLoopRampRate(1);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.closedLoop.p(shooterkPPreference.get());
    config.closedLoop.d(shooterkDPreference.get());
    config.closedLoop.i(shooterkIPreference.get());
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.outputRange(
        ShooterConstants.ROLLER_MIN_OUTPUT, ShooterConstants.ROLLER_MAX_OUTPUT);
    config.closedLoop.feedForward.kV(shooterkFPreference.get());
  }

  private void configureIndexMotor(SparkMax motor) {
    SparkMaxConfig config = new SparkMaxConfig();
    // configure indexer motor
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    // TODO: Make actual constants
    config.smartCurrentLimit(40);
    config.closedLoopRampRate(0);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configurePositionMotor(
      TalonFX motor,
      TalonFXConfiguration baseConfiguration,
      MotorOutputConfigs motorOutputConfigs,
      Slot0Configs slot0PID,
      SoftwareLimitSwitchConfigs limitSwitchConfigs,
      MotionMagicConfigs mmConfig) {

    TalonFXConfigurator configurator = m_shooterPositionMotor.getConfigurator();

    // grab any settings already on the motor, just in case.
    configurator.refresh(baseConfiguration);
    configurator.refresh(motorOutputConfigs);
    configurator.refresh(slot0PID);
    configurator.refresh(limitSwitchConfigs);
    configurator.refresh(mmConfig);

    baseConfiguration.Feedback.FeedbackRemoteSensorID = m_shooterPositionCancoder.getDeviceID();
    baseConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    configurator.apply(baseConfiguration);

    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    configurator.apply(motorOutputConfigs, 0.050);

    slot0PID.kV = ShooterConstants.POSITION_kV;
    slot0PID.kS = ShooterConstants.POSITION_kS;
    slot0PID.kP = ShooterConstants.POSITION_kP;
    slot0PID.kI = ShooterConstants.POSITION_kI;
    slot0PID.kD = ShooterConstants.POSITION_kD;

    configurator.apply(slot0PID, 0.050);

    limitSwitchConfigs.ForwardSoftLimitEnable = true;
    limitSwitchConfigs.ForwardSoftLimitThreshold = ShooterConstants.POSITION_ForwardsLimit;
    limitSwitchConfigs.ReverseSoftLimitEnable = true;
    limitSwitchConfigs.ReverseSoftLimitThreshold = ShooterConstants.POSITION_ReverseLimit;

    configurator.apply(limitSwitchConfigs, 0.050);

    mmConfig.MotionMagicCruiseVelocity = ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    mmConfig.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;
    mmConfig.MotionMagicJerk = ShooterConstants.MOTION_MAGIC_JERK;

    configurator.apply(mmConfig);
  }

  private void configureCancoder(CANcoder cancoder) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    config.MagnetSensor.MagnetOffset = 0;
    cancoder.getConfigurator().apply(config);
    cancoder.getAbsolutePosition().setUpdateFrequency(100);
    cancoder.getPosition().setUpdateFrequency(100);
    cancoder.getVelocity().setUpdateFrequency(100);
    cancoder.setPosition(degreesToCANcoder(0.5, ShooterConstants.ARM_CANCODER_RATIO));
  }

  public void spinPower(double power) {
    m_shooterMotor.set(
        MathUtil.clamp(
            power, ShooterConstants.ROLLER_MIN_OUTPUT, ShooterConstants.ROLLER_MAX_OUTPUT));
  }

  public void spinRPM(double rpm) {
    targetVelocity = rpm;
    m_RollerPidController.setReference(
        MathUtil.clamp(rpm, -ShooterConstants.ROLLER_MAX_RPM, ShooterConstants.ROLLER_MAX_RPM),
        SparkFlex.ControlType.kVelocity);
  }

  @Logged
  public boolean atRPM() {
    if (Robot.isSimulation()) return true;
    return velocity >= targetVelocity - ShooterConstants.VELOCITY_TOLERANCE
        && velocity <= targetVelocity + ShooterConstants.VELOCITY_TOLERANCE;
  }

  public void runIndex(double power) {
    m_shooterIndexMotor.set(MathUtil.clamp(power, -1, 1));
  }

  // public double getPositionRevolutions(){
  //   return m_shooterPositionMotor.getPosition().getValueAsDouble();
  // }

  // @Log.File
  // @Log.NT
  // public double getRotorAngleDegrees(){
  //   return
  // m_shooterPositionMotor.getPosition().getValueAsDouble()*ShooterConstants.POSITION_DEGREE_PER_MOTOR_REV;
  // }

  public double getAngleRadians() {
    return Units.degreesToRadians(getAngleDegrees());
  }

  @Logged
  public double getAngleDegrees() {
    return CANcoderToDegrees(
        m_shooterPositionCancoder.getAbsolutePosition().getValueAsDouble(),
        ShooterConstants.ARM_CANCODER_RATIO);
  }

  @Logged
  public double getShootingVelocity() {
    return velocity + robotVelocity;
  }

  private void setPositionRevolutions(double position) {
    m_shooterPositionMotor.setControl(shooterPosition.withPosition(position));
    // m_shooterPositionMotor.setControl(mmDutyCycleRequest.withPosition(position));
  }

  public void setAngleDegrees(double angle) {
    this.targetPosition = angle;
    // setPositionRevolutions(angle.ShooterConstants.POSITION_DEGREE_PER_MOTOR_REV);
    setPositionRevolutions(degreesToCANcoder(angle, ShooterConstants.ARM_CANCODER_RATIO));
  }

  @Logged
  public boolean armAtSetpoint() {
    if (Robot.isSimulation()) return true;
    return getAngleDegrees() >= targetPosition - ShooterConstants.POSITION_TOLERANCE
        && getAngleDegrees() <= targetPosition + ShooterConstants.POSITION_TOLERANCE;
  }

  public void stopRoller() {
    m_shooterMotor.stopMotor();
  }

  public void moveArm(double motorpower) {
    m_shooterPositionMotor.set(MathUtil.clamp(motorpower, -1, 1));
  }

  public void stopIndexer() {
    m_shooterIndexMotor.stopMotor();
  }

  public void stopPositioner() {
    m_shooterPositionMotor.stopMotor();
  }

  public void resetPosition() {
    this.setAngleDegrees(0);
  }

  /**
   * @param positionCounts CANCoder Position Counts
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
    // return positionCounts * (360.0 / (gearRatio * 4096.0));
    return positionCounts * 360 / gearRatio;
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return CANCoder Position Counts
   */
  public static double degreesToCANcoder(double degrees, double gearRatio) {
    return degrees * gearRatio / 360.0;
  }

  public void stopAll() {
    this.stopRoller();
    this.stopIndexer();
    this.resetPosition();
  }

  @Logged
  public boolean getIndexerBeamBreak() {
    return !m_indexerBeamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    temp = m_shooterMotor.getMotorTemperature();
    velocity = m_shooterEncoder.getVelocity();
    current = m_shooterMotor.getOutputCurrent();

    tempIndex = m_shooterIndexMotor.getMotorTemperature();
    velocityIndex = m_shooterIndexEncoder.getVelocity();
    currentIndex = m_shooterIndexMotor.getOutputCurrent();

    armPower = m_shooterPositionMotor.get();
    armPosition = m_shooterPositionMotor.getPosition().getValueAsDouble();
    armVelocity = m_shooterPositionMotor.getVelocity().getValueAsDouble();
    armVoltage = m_shooterPositionMotor.getMotorVoltage().getValueAsDouble();
    armTemp = m_shooterPositionMotor.getDeviceTemp().getValueAsDouble();
    armCurrent = m_shooterPositionMotor.getTorqueCurrent().getValueAsDouble();
    armCancoderPosition = m_shooterPositionCancoder.getAbsolutePosition().getValueAsDouble();
    armCancoderVelocity = m_shooterPositionCancoder.getVelocity().getValueAsDouble();

    // m_shooterPositionMotor.getConfigurator().refresh(fxCfg);
    // armNeutralMode = fxCfg.MotorOutput.NeutralMode.toString();
    // armCurrentFault = m_shooterPositionMotor.getFault_StatorCurrLimit().getValue();
    // armRevLimiFault = m_shooterPositionMotor.getFault_ReverseSoftLimit().getValue();

    // TODO remove once tuned.
    // m_RollerPidController.setP(shooterkPPreference.get());
    // m_RollerPidController.setI(shooterkIPreference.get());
    // m_RollerPidController.setD(shooterkDPreference.get());
    // m_RollerPidController.setFF(shooterkFPreference.get());

    // apply gains, 50 ms total timeout
    // TODO remove once tuned.
    // positionSlot0Configs.kP = positionkPPreference.get();
    // positionSlot0Configs.kI = positionkIPreference.get();
    // positionSlot0Configs.kD = positionkDPreference.get();

    // m_shooterPositionMotor.getConfigurator().apply(positionSlot0Configs, 0.050);
  }

  public void simulationPeriodic() {
    m_shooterPositionMotor.setPosition(targetPosition);
  }

  public void setReady(boolean bool) {
    Ready = bool;
  }

  public boolean getReady() {
    return Ready;
  }

  // position 0.0-1.0 (one extreme to other extreme)
  public void setServoPosition(double position) {
    m_RightServo.set(position);
    m_LeftServo.set(1.0 - position);
  }

  public void resetServoPosition() {
    m_RightServo.set(0);
    m_LeftServo.set(1);
  }
}

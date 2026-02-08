package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;

// Steer Motor Feedback and Feedforward configs
final class SteerMotorConfigs {

  private SteerMotorConfigs() {}

  // SYS ID TIMEOUTs
  protected static final int QUASISTATIC_TIMEOUT = 5;
  protected static final int DYNAMIC_TIMEOUT = 3;

  // FRONT LEFT
  protected static final double FL_STEER_KP = 27.807;
  protected static final double FL_STEER_KD = 0.54932;
  protected static final double FL_STEER_KS = 0.20113;
  protected static final double FL_STEER_KV = 1.1265;
  protected static final double FL_STEER_KA = 0.016335;

  protected static Slot0Configs createFrontLeftSteerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = FL_STEER_KP;
    slot.kD = FL_STEER_KD;
    slot.kS = FL_STEER_KS;
    slot.kV = FL_STEER_KV;
    slot.kA = FL_STEER_KA;
    return slot;
  }

  // FRONT RIGHT
  protected static final double FR_STEER_KP = 49.731;
  protected static final double FR_STEER_KD = 1.3438;
  protected static final double FR_STEER_KS = 0.23952;
  protected static final double FR_STEER_KV = 1.1515;
  protected static final double FR_STEER_KA = 0.014836;

  protected static Slot0Configs createFrontRightSteerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = FR_STEER_KP;
    slot.kD = FR_STEER_KD;
    slot.kS = FR_STEER_KS;
    slot.kV = FR_STEER_KV;
    slot.kA = FR_STEER_KA;
    return slot;
  }

  // REAR LEFT
  protected static final double RL_STEER_KP = 21.938;
  protected static final double RL_STEER_KD = 0.27919;
  protected static final double RL_STEER_KS = 0.30556;
  protected static final double RL_STEER_KV = 1.1571;
  protected static final double RL_STEER_KA = 0.01352;

  protected static Slot0Configs createRearLeftSteerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = RL_STEER_KP;
    slot.kD = RL_STEER_KD;
    slot.kS = RL_STEER_KS;
    slot.kV = RL_STEER_KV;
    slot.kA = RL_STEER_KA;
    return slot;
  }

  // REAR RIGHT
  protected static final double RR_STEER_KP = 27.339;
  protected static final double RR_STEER_KD = 0.52467;
  protected static final double RR_STEER_KS = 0.21727;
  protected static final double RR_STEER_KV = 1.1237;
  protected static final double RR_STEER_KA = 0.015932;

  protected static Slot0Configs createRearRightSteerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = RR_STEER_KP;
    slot.kD = RR_STEER_KD;
    slot.kS = RR_STEER_KS;
    slot.kV = RR_STEER_KV;
    slot.kA = RR_STEER_KA;
    return slot;
  }
}

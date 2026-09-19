// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

// import frc.utils.InterpolatingCalculator;
// import frc.utils.InterParameter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class ShooterConstants {

  // servos
  public static final int RIGHT_SERVO_PORT = 0;
  public static final int LEFT_SERVO_PORT = 1;

  // public static final double POSITION_kF = 1;
  public static final double POSITION_kP = 60;
  public static final double POSITION_kI = 0;
  public static final double POSITION_kD = 2;
  public static final double POSITION_kS = 0.24;
  public static final double POSITION_kV = 0.12;
  public static final double POSITION_ForwardsLimit = 0.42;
  public static final double POSITION_ReverseLimit = 0;

  public static final double ROLLER_kP = 9e-5;
  public static final double ROLLER_kI = 0;
  public static final double ROLLER_kD = 0.001;
  public static final double ROLLER_kF = 0.0001975;
  public static final double ROLLER_MAX_OUTPUT = 1;
  public static final double ROLLER_MIN_OUTPUT = -1;
  public static final double ROLLER_MAX_RPM = 5700;

  public static final double Index_IntakeSpeed = 0.2;

  public static final double TARGET_POSITION_DEGREES = 56;
  public static final double SHOOTER_HOME_DEGREES = 2;
  public static final double ARM_CANCODER_RATIO = 5.0 / 3.0;

  // DISTANCE, ANGLE, RPM
  // public static final InterCalculator SHOOTER_INTER_CALCULATOR = new InterCalculator(
  //   new InterParameter(1.5, 42, 4000),
  //   new InterParameter(2, 35, 4000),
  //   new InterParameter(2.25, 35, 4000),
  //   new InterParameter(2.5, 30, 4000),
  //   new InterParameter(2.75, 30, 4000),
  //   new InterParameter(3.0, 27, 4000),
  //   new InterParameter(3.25, 25, 4000),
  //   new InterParameter(3.5, 24, 4000),
  //   new InterParameter(3.75, 23, 4000),
  //   new InterParameter(4.0, 22, 4000),
  //   new InterParameter(4.25, 21, 4000),
  //   new InterParameter(4.5, 20, 4000),
  //   new InterParameter(4.75, 19, 4000)
  //   );

  // RED 4
  // "x": 16.579342,
  // "y": 5.547867999999999,
  // BLUE 7
  // "x": -0.038099999999999995,
  // "y": 5.547867999999999,
  public static final int RED_SPEAKER_ID = 4;
  public static final Pose2d RED_SPEAKER = new Pose2d(16.58, 5.55, Rotation2d.fromDegrees(0));
  public static final int BLUE_SPEAKER_ID = 7;
  public static final Pose2d BLUE_SPEAKER = new Pose2d(0, 5.55, Rotation2d.fromDegrees(180));

  // TODO: TUNE
  public static final double POSITION_TOLERANCE = 1.0;

  public static final double VELOCITY_TOLERANCE = 75;

  public static final double LENGTH = 1.5;

  // 100 to 1 gear ration
  public static final double POSITION_DEGREE_PER_MOTOR_REV = 360.0 / 100.0;

  public static final double ELEVATION = Units.inchesToMeters(9.46);

  public static final double TRANSLATION_OFFSET = Units.inchesToMeters(1.66);

  // shooter motion magic constants
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 400;
  public static final double MOTION_MAGIC_ACCELERATION = 1000; // full speed in .5 seconds
  public static final double MOTION_MAGIC_JERK = 2000;

  public static final double AUTO_TARGET_ROT_kP = 0.014;
  public static final double AUTO_TARGET_ROT_kD = 0.0;

  public static final double kFreeSpeedRpm = 5676;

  public static final int SHOOTER_POSITION_MOTOR = 7;
  public static final int SHOOTER_MOTOR_LEADERCanId = 8;
  public static final int SHOOTER_INDEX_MOTOR = 9;
  public static final int SHOOTER_POSITION_CANCODER = 20;
}

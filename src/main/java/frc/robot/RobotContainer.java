// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootPiece;
import frc.robot.preferences.DoublePreference;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // already logged with telemetry class
  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // The controllers are defined here
  private static final CommandXboxController joystick = new CommandXboxController(0);

  private final Telemetry logger = new Telemetry(DriveConstants.MAX_DRIVE_SPEED);

  private DoublePreference shooterIndexPower =
      new DoublePreference("shooter/ShootingIndexPower", 0.5);

  // Cannd shot angles
  private DoublePreference wingShotAngle = new DoublePreference("shooter/wingShotAngle", 34);
  private DoublePreference podiumShotAngle = new DoublePreference("shooter/podiumShotAngle", 52.5);
  private DoublePreference subShotAngle = new DoublePreference("shooter/subShotAngle", 93);

  // Canned shot RPM
  private DoublePreference wingShotRPM = new DoublePreference("shooter/wingRPM", 4000);
  private DoublePreference podiumShotRPM = new DoublePreference("shooter/podiumRPM", 3200);
  private DoublePreference subShotRPM = new DoublePreference("shooter/subRPM", 3200);

  private final Command shootSubwoofer =
      new ShootPiece(
          m_shooter,
          subShotAngle,
          subShotRPM,
          shooterIndexPower,
          () -> joystick.leftTrigger().getAsBoolean());
  private final Command shootPodium =
      new ShootPiece(
          m_shooter,
          podiumShotAngle,
          podiumShotRPM,
          shooterIndexPower,
          () -> joystick.leftTrigger().getAsBoolean());
  private final Command shootWing =
      new ShootPiece(
          m_shooter,
          wingShotAngle,
          wingShotRPM,
          shooterIndexPower,
          () -> joystick.leftTrigger().getAsBoolean());

  // private static JoystickButton driver_x = new JoystickButton(joystick,
  // XboxController.Button.kX.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    configureSubsystemDefaultCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    joystick.a().whileTrue(shootSubwoofer);
    joystick.b().whileTrue(shootPodium);
    joystick.y().whileTrue(shootWing);
    /*
    joystick.x().onTrue(drivetrain.sysIdSteer());
    joystick.y().onTrue(drivetrain.sysIdTranslation());
    joystick.a().onTrue(drivetrain.driveForward());
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    drivetrain.registerTelemetry(logger::telemeterize);
    */
  }

  // Subsystem Default Commands
  private void configureSubsystemDefaultCommands() {

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                DriveConstants.DEFAULT_DRIVE_REQUEST
                    .withVelocityX(
                        -1
                            * Math.copySign(Math.pow(joystick.getLeftY(), 2), joystick.getLeftY())
                            * DriveConstants
                                .MAX_DRIVE_SPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -1
                            * Math.copySign(Math.pow(joystick.getLeftX(), 2), joystick.getLeftX())
                            * DriveConstants.MAX_DRIVE_SPEED) // Drive left with negative X (left)
                    .withRotationalRate(
                        -1
                            * Math.copySign(Math.pow(joystick.getRightX(), 2), joystick.getRightX())
                            * DriveConstants
                                .MAX_ANGULAR_SPEED) // Drive counterclockwise with negative X (left)
            ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}

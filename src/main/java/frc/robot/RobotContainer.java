// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  // The controllers are defined here
  private static final CommandXboxController joystick = new CommandXboxController(0);

  //private static JoystickButton driver_x = new JoystickButton(joystick, XboxController.Button.kX.value);

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
    joystick.x().onTrue(drivetrain.sysIdSteer());
  }

  //Subsystem Default Commands
  private void configureSubsystemDefaultCommands(){

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
        DriveConstants.DEFAULT_DRIVE_REQUEST.withVelocityX(-1 * Math.copySign(Math.pow(joystick.getLeftY(),2), joystick.getLeftY()) * DriveConstants.MAX_DRIVE_SPEED) // Drive forward with negative Y (forward)
          .withVelocityY(-1 * Math.copySign(Math.pow(joystick.getLeftX(), 2), joystick.getLeftX()) * DriveConstants.MAX_DRIVE_SPEED) // Drive left with negative X (left)
          .withRotationalRate(-1 * Math.copySign(Math.pow(joystick.getRightX(), 2), joystick.getRightX()) * DriveConstants.MAX_ANGULAR_SPEED) // Drive counterclockwise with negative X (left)
        )
    );

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

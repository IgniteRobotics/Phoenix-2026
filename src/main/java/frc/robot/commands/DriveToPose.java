package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class DriveToPose extends Command {
  private DrivetrainSubsystem drivetrain;
  private Pose2d targetPose;
  private PIDController xControl;
  private PIDController yControl;
  private PIDController rotControl;

  public DriveToPose(DrivetrainSubsystem subsystem, Pose2d pose) {
    drivetrain = subsystem;
    targetPose = pose;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xControl = drivetrain.getTranslationPIDController();
    yControl = drivetrain.getTranslationPIDController();
    rotControl = drivetrain.getRotationPIDController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(
        DriveConstants.AUTO_DRIVE_REQUEST
            .withVelocityX(xControl.calculate(drivetrain.getFieldPose().getX(), targetPose.getX()))
            .withVelocityY(yControl.calculate(drivetrain.getFieldPose().getY(), targetPose.getY()))
            .withRotationalRate(
                rotControl.calculate(
                    drivetrain.getFieldPose().getRotation().getDegrees(),
                    targetPose.getRotation().getDegrees())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
        DriveConstants.AUTO_DRIVE_REQUEST.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xControl.atSetpoint() && yControl.atSetpoint() && rotControl.atSetpoint();
  }
}

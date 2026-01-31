package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.statemachines.DriveState;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;

public class DrivetrainSubsystem extends CommandSwerveDrivetrain {
  private DriveState driveState = DriveState.getInstance();
  public DrivetrainSubsystem() {
    super(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
  }

  @Override
  public void periodic() {
    super.periodic();
    for (VisionMeasurement estimate :
        driveState.grabVisionEstimateList(CameraConstants.photonCameraName_FrontLeft)) {
      addVisionMeasurement(
          estimate.getEstimatedPose().estimatedPose.toPose2d(),
          estimate.getTimestamp(),
          estimate.getTrust());
    }
    for (VisionMeasurement estimate :
        driveState.grabVisionEstimateList(CameraConstants.photonCameraName_FrontRight)) {
      addVisionMeasurement(
          estimate.getEstimatedPose().estimatedPose.toPose2d(),
          estimate.getTimestamp(),
          estimate.getTrust());
    }
    driveState.adjustCurrentDriveStats(this.getStateCopy());
    SmartDashboard.putNumber("Robot Pose X", this.getFieldPose().getX());
    SmartDashboard.putNumber("Robot Pose Y", this.getFieldPose().getY());
  }

  public PIDController getTranslationPIDController(){
    PIDController controller = new PIDController(DrivePreferences.translation_kP.getValue(), 0, DrivePreferences.translation_kD.getValue());
    controller.setTolerance(DriveConstants.TRANSLATION_ALIGN_TOLERANCE);
    return controller;
  }

  public PIDController getRotationPIDController(){
    PIDController controller = new PIDController(DrivePreferences.rotation_kP.getValue(), 0, DrivePreferences.rotation_kD.getValue());
    controller.setTolerance(DriveConstants.ROTATION_ALIGN_TOLERANCE);
    controller.enableContinuousInput(-180, 180); //TODO:Determine if input is in radians or degrees
    return controller;
  }

  public Pose2d getFieldPose(){
    return this.getState().Pose;
  }

}

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
    applySteerGains();
  }
  
  private void applySteerGains(){
    this.getModule(0).getSteerMotor().getConfigurator().apply(SteerConstants.createFrontLeftSteerMotorSlot0Configs());
    this.getModule(1).getSteerMotor().getConfigurator().apply(SteerConstants.createFrontRightSteerMotorSlot0Configs());
    this.getModule(2).getSteerMotor().getConfigurator().apply(SteerConstants.createRearLeftSteerMotorSlot0Configs());
    this.getModule(3).getSteerMotor().getConfigurator().apply(SteerConstants.createRearRightSteerMotorSlot0Configs());
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
  }

  public Command sysIdSteer(){
    return m_sysIdRoutineSteer.quasistatic(Direction.kForward).withTimeout(10).andThen(m_sysIdRoutineSteer.quasistatic(Direction.kReverse).withTimeout(10))
            .andThen(m_sysIdRoutineSteer.dynamic(Direction.kForward).withTimeout(10)).andThen(m_sysIdRoutineSteer.dynamic(Direction.kReverse).withTimeout(10));
  }
}

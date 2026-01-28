package frc.robot.subsystems.drive;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    return new InstantCommand(() -> SignalLogger.start()).andThen(m_sysIdRoutineSteer.quasistatic(Direction.kForward).withTimeout(5)).andThen(m_sysIdRoutineSteer.quasistatic(Direction.kReverse).withTimeout(5))
            .andThen(m_sysIdRoutineSteer.dynamic(Direction.kForward).withTimeout(3)).andThen(m_sysIdRoutineSteer.dynamic(Direction.kReverse).withTimeout(3)).andThen(new WaitCommand(5)).andThen(new InstantCommand(() -> SignalLogger.stop()));
  }
}

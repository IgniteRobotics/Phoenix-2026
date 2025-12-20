package frc.robot.subsystems.drive;

import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class DrivetrainSubsystem extends CommandSwerveDrivetrain {
  public DrivetrainSubsystem() {
    super(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
  }

  @Override
  public void periodic() {super.periodic();}
}

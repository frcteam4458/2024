package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class B1R3_2Note extends SwerveTrajectoryCommand {

  public B1R3_2Note(DriveSubsystem driveSubsystem) {
    super(driveSubsystem, getAutonomousTrajectory());
  }

  public static String getAutonomousTrajectory() {
   return "auto0";
  }
}

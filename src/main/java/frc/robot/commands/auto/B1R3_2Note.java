package frc.robot.commands.auto;

import frc.robot.subsystems.drive.DriveSubsystem;

/** Add your docs here. */
public class B1R3_2Note extends SwerveTrajectoryCommand {

  public B1R3_2Note(DriveSubsystem driveSubsystem) {
    super(driveSubsystem, getAutonomousTrajectory());
  }

  public static String getAutonomousTrajectory() {
   return "auto0";
  }
}

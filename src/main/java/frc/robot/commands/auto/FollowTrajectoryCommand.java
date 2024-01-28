package frc.robot.commands.auto;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

// Get rid of this class
public class FollowTrajectoryCommand extends RamseteCommand {

  public static TrajectoryConfig trajectoryConfig =
      new TrajectoryConfig(ControlConstants.kMaxAutoVel, ControlConstants.kMaxAutoAccel)
          .addConstraint(
              new DifferentialDriveVoltageConstraint(
                  new SimpleMotorFeedforward(HardwareConstants.kS, HardwareConstants.kV),
                  OperatorConstants.kinematics,
                  6))
          .setKinematics(OperatorConstants.kinematics);

  DriveSubsystem driveSubsystem;

  public FollowTrajectoryCommand(DriveSubsystem driveSubsystem, Trajectory trajectory) {
    super(
        trajectory,
        driveSubsystem::getPose,
        new RamseteController(ControlConstants.kB, ControlConstants.kZeta),
        OperatorConstants.kinematics,
        driveSubsystem::setWheelSpeeds,
        driveSubsystem);
  }
}

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTrajectoryCommand extends SequentialCommandGroup {

  DriveSubsystem driveSubsystem;

  /**
   * Runs an autonomous PathPlanner routine.
   * @param driveSubsystem
   * @param path Name of the auto
   */
  public SwerveTrajectoryCommand(DriveSubsystem driveSubsystem, String path) {
    this(driveSubsystem, path, true);
  }

  /**
   * Runs a PathPlanner path or autonomous routine
   * @param driveSubsystem
   * @param path Name of path/auto
   * @param auto Whether or not the name of routine is a path or an autonomous
   */
  public SwerveTrajectoryCommand(DriveSubsystem driveSubsystem, String path, boolean auto) {

    if(auto) {
      driveSubsystem.driveChassisSpeeds(new ChassisSpeeds(0, 0, 0));
      addCommands(AutoBuilder.buildAuto(path));
    } else {
      addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(path)));
    }
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  /**
   * Pathfind to pose.
   * @param driveSubsystem
   * @param pose The pose to pathfind to
   * @param constraints Velocity/acceleration constraints
   * @param goalEndVel Velocity that the robot should be moving at when the command ends
   * @param rotationDelayDistance Distance robot should move before initiating a turn
   */
  public SwerveTrajectoryCommand(DriveSubsystem driveSubsystem, Pose2d pose,
  PathConstraints constraints, double goalEndVel, double rotationDelayDistance) {
  addCommands(AutoBuilder.pathfindToPose(pose, constraints, goalEndVel, rotationDelayDistance));
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  /**
   * Simplified pathfinding constructor
   * @param driveSubsystem
   * @param pose
   * @param goalEndVel Velocity robot should be moving at when command ends
   */
  public SwerveTrajectoryCommand(DriveSubsystem driveSubsystem, Pose2d pose, double goalEndVel) {
    this(driveSubsystem, pose,
      new PathConstraints(OperatorConstants.kMaxSpeed,
        1.0,
        6.28,
        6.28
      ),
      goalEndVel,
      0
    );
  }

  /**
   * yep
   * @param driveSubsystem
   * @param pose
   * @param constraints
   * @param goalEndVel
   * @param rotationDelayDistance
   * @param path
   * 
   * TODO: Alternate constructor that instead of Pose2d, gets the starting pose of the preplanned path and automatically pathfinds to that pose
   */
  public SwerveTrajectoryCommand(DriveSubsystem driveSubsystem,
    Pose2d pose,
    PathConstraints constraints,
    double goalEndVel,
    double rotationDelayDistance,
    String path)
  {
    this(driveSubsystem, pose, constraints, goalEndVel, rotationDelayDistance);
    addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(path)));
  }
}

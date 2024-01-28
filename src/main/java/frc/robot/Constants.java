package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    // User controller settings
    public static final int kDriverControllerPort = 0;
    public static final int kOperartorControllerPort = 1;
    public static final double kDeadzone = 0.05;

    public static final double kDriveRateLimit = 2.0;
    public static final double kDriveSpeedDivisor = 1.5;
    public static final double kDriveTurnDivisor = 2.5;

    public static final DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(HardwareConstants.kTrackWidth);
  }

  public static class VisionConstants {
    public static final Transform3d kTransformToRobot =
        new Transform3d(new Translation3d(0, 0, 1), new Rotation3d());
  }

  public static class PositionConstants {
    public static final Pose2d kAmpPose = new Pose2d(1.85, 7.7, Rotation2d.fromDegrees(270));
    public static final double kAmpArmAngle = 1.640;

    public static final Rotation2d kSourceRotation = Rotation2d.fromRadians(-2.075);
    public static final Pose2d kSource1Pose = new Pose2d(1.75, 0.675, kSourceRotation);
    public static final Pose2d kSource2Pose = new Pose2d(1.2, 1.0, kSourceRotation);
    public static final Pose2d kSource3Pose = new Pose2d(0.6, 1.33, kSourceRotation);

    public static final double kSourceArmAngle = 0.84;

    public static final double kArmFloorAngle = 0.0;

    public static final Translation2d kSpeakerPosition = new Translation2d(0.5, 5.5);
  }

  public static class ControlConstants {
    public static final double kP = 5.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kAP = 5.0;
    public static final double kAI = 0.0;
    public static final double kAD = 0.0;

    public static final double kArmP = 50.0;
    public static final double kArmI = 0.0;
    public static final double kArmD = 0.0;
    public static final double kArmRadPerSec = Math.toRadians(180);
    public static final double kArmAccel = Math.toRadians(720);

    public static final boolean kArmPid = true;

    // Ramsete settings
    public static final double kB = 2.0;
    public static final double kZeta = 0.7;

    // Auto velocity/accel config
    public static final double kMaxAutoVel = 2.0;
    public static final double kMaxAutoAccel = 1.0;
  }

  public static class HardwareConstants {
    public static final int kArmMotor = 9;
    public static final int kArmMotorFollower = 10;

    public static final double kArmPositionConversionFactor = (1.0/200.0) * 2 * Math.PI;
    public static final double kArmMass = Units.lbsToKilograms(21.853);
    public static final double kArmLength = Units.inchesToMeters(29.240);
    public static final double kYOriginToArm = 0.203;
    public static final double kZOriginToArm = 0.279;

    public static final double kArmRotPhysicalMin = -0.02;
    public static final double kArmRotPhysicalMax = 3.429;

    // Drivetrain Limits
    public static final double kMaxSpeed = 3.81;
    public static final double kMaxAngVel = 12; // experimental value
    public static final double kDriveBaseRadius = Units.inchesToMeters(Math.sqrt(Math.pow(11.75, 2.0) + Math.pow(10.75, 2.0))); // This is wrong
    public static final double kDriveGearing = 10.71; // this is just not true anymore, but for YAGSL gearing is stored in the deploy json
    public static final double kMOI = 7.5; // unmeasured, arbitrary value
    public static final double kMass = 60.0; // ^

    // Encoder Conversions
    public static final double kDistancePerPulse = (.1524 * Math.PI) / 10.71;
    public static final double kVelocityConversionFactor = 0.0865911847 / 60.0;

    // Measured in SysID on 2023 robot
    public static final double kS = 0.120;
    public static final double kV = 1.36;
    public static final double kA = 0.236;
    public static final double kP = 0.146;
    public static final double kAngularP = 0.146; // arbitrary
    public static final double kTrackWidth = .7112;
  }
}

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface DriveSubsystemIO {

  @AutoLog
  public static class DriveSubsystemIOInputs {
    public double flPosition = 0.0;
    public double flVelocity = 0.0;
    public double flVolts = 0.0;
    public double flAmps = 0.0;

    public double frPosition = 0.0;
    public double frVelocity = 0.0;
    public double frVolts = 0.0;
    public double frAmps = 0.0;

    public double blPosition = 0.0;
    public double blVelocity = 0.0;
    public double blVolts = 0.0;
    public double blAmps = 0.0;

    public double brPosition = 0.0;
    public double brVelocity = 0.0;
    public double brVolts = 0.0;
    public double brAmps = 0.0;

    public double flAnglePosition = 0.0;
    public double flAngleVelocity = 0.0;
    public double flAngleVolts = 0.0;
    public double flAngleAmps = 0.0;
    public double flAngleAbsolute = 0.0;

    public double frAnglePosition = 0.0;
    public double frAngleVelocity = 0.0;
    public double frAngleVolts = 0.0;
    public double frAngleAmps = 0.0;
    public double frAngleAbsolute = 0.0;

    public double blAnglePosition = 0.0;
    public double blAngleVelocity = 0.0;
    public double blAngleVolts = 0.0;
    public double blAngleAmps = 0.0;
    public double blAngleAbsolute = 0.0;

    public double brAnglePosition = 0.0;
    public double brAngleVelocity = 0.0;
    public double brAngleVolts = 0.0;
    public double brAngleAmps = 0.0;
    public double brAngleAbsolute = 0.0;

    public double leftEncoderAverage = 0.0;
    public double rightEncoderAverage = 0.0;

    public double gyroYaw = 0.0;

    public Pose2d robotPose = new Pose2d();
  }

  public default void updateInputs(DriveSubsystemIOInputs inputs) {}

  public default void setChassisSpeeds(ChassisSpeeds speeds) {}
  public default void drive(ChassisSpeeds speeds) {}
  public default void driveFieldOriented(ChassisSpeeds speeds) {}

  public default ChassisSpeeds getChassisSpeeds() {
    return null;
  }

  public default void addVisionMeasurement(Pose3d pose, double timestamp) {}

  public default void setPose(Pose2d pose) {}

  public default void setVolts(double left, double right) {}
  public default void setCoast(boolean coast) {}

  public default void resetGyro(Rotation3d rotation) {}
}

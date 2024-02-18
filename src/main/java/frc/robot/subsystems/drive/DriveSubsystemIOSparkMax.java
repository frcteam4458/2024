package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.HardwareConstants;

import java.io.File;
import java.io.IOException;

import org.littletonrobotics.junction.Logger;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class DriveSubsystemIOSparkMax implements DriveSubsystemIO {
  SwerveDrive swerveDrive;

  // CANSparkMax fl, fr, bl, br;
  // RelativeEncoder encoderFl, encoderFr, encoderBl, encoderBr;
  // MotorControllerGroup left, right;
  // DifferentialDrive drive;

  // PigeonIMU pigeon;

  public DriveSubsystemIOSparkMax() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(HardwareConstants.kMaxSpeed);
    } catch (IOException e) {
      e.printStackTrace();
    }
    System.out.println(swerveDrive.getSwerveController().config.maxAngularVelocity);

    for (int i = 0; i < 4; i++) {
      // swerveDrive.getModules()[i].setDriveMotorConversionFactor(0.0392120164);
      // swerveDrive.getModules()[i].setAngleMotorConversionFactor(28.125);
    }
    // swerveDrive.setHeadingCorrection(true);
    // SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.NONE;
  }

  @Override
  public void updateInputs(DriveSubsystemIOInputs inputs) {
    double voltage = RobotController.getBatteryVoltage();

    inputs.flPosition = swerveDrive.getModules()[0].getDriveMotor().getPosition();
    inputs.flVelocity = swerveDrive.getModules()[0].getDriveMotor().getVelocity();
    inputs.flVolts = getMotor(0, 0).get() * voltage;
    inputs.flAmps = getMotor(0, 0).getOutputCurrent();

    inputs.frPosition = swerveDrive.getModules()[1].getDriveMotor().getPosition();
    inputs.frVelocity = swerveDrive.getModules()[1].getDriveMotor().getVelocity();
    inputs.frVolts = getMotor(1, 0).get() * voltage;
    inputs.frAmps = getMotor(1, 0).getOutputCurrent();

    inputs.blPosition = swerveDrive.getModules()[2].getDriveMotor().getPosition();
    inputs.blVelocity = swerveDrive.getModules()[2].getDriveMotor().getVelocity();
    inputs.blVolts = getMotor(2, 0).get() * voltage;
    inputs.blAmps = getMotor(2, 0).getOutputCurrent();

    inputs.brPosition = swerveDrive.getModules()[3].getDriveMotor().getPosition();
    inputs.brVelocity = swerveDrive.getModules()[3].getDriveMotor().getVelocity();
    inputs.brVolts = getMotor(3, 0).get() * voltage;
    inputs.brAmps = getMotor(3, 0).getOutputCurrent();

    inputs.flAnglePosition = swerveDrive.getModules()[0].getAngleMotor().getPosition();
    inputs.flAngleVelocity = swerveDrive.getModules()[0].getAngleMotor().getVelocity();
    inputs.flAngleVolts = getMotor(0, 1).get() * voltage;
    inputs.flAngleAmps = getMotor(0, 1).getOutputCurrent();
    inputs.flAngleAbsolute = swerveDrive.getModules()[0].getAbsolutePosition();

    inputs.frAnglePosition = swerveDrive.getModules()[1].getAngleMotor().getPosition();
    inputs.frAngleVelocity = swerveDrive.getModules()[1].getAngleMotor().getVelocity();
    inputs.frAngleVolts = getMotor(1, 1).get() * voltage;
    inputs.frAngleAmps = getMotor(1, 1).getOutputCurrent();
    inputs.frAngleAbsolute = swerveDrive.getModules()[1].getAbsolutePosition();

    inputs.blAnglePosition = swerveDrive.getModules()[2].getAngleMotor().getPosition();
    inputs.blAngleVelocity = swerveDrive.getModules()[2].getAngleMotor().getVelocity();
    inputs.blAngleVolts = getMotor(2, 1).get() * voltage;
    inputs.blAngleAmps = getMotor(2, 1).getOutputCurrent();
    inputs.blAngleAbsolute = swerveDrive.getModules()[2].getAbsolutePosition();

    inputs.brAnglePosition = swerveDrive.getModules()[3].getAngleMotor().getPosition();
    inputs.brAngleVelocity = swerveDrive.getModules()[3].getAngleMotor().getVelocity();
    inputs.brAngleVolts = getMotor(3, 1).get() * voltage;
    inputs.brAngleAmps = getMotor(3, 1).getOutputCurrent();
    inputs.brAngleAbsolute = swerveDrive.getModules()[3].getAbsolutePosition();

    inputs.leftEncoderAverage = (inputs.flPosition + inputs.blPosition) / 2.0;
    inputs.rightEncoderAverage = (inputs.frPosition + inputs.brPosition) / 2.0;

    inputs.gyroYaw = swerveDrive.getYaw().getDegrees();

    inputs.robotPose = swerveDrive.getPose();

    Logger.recordOutput("Modules", new double[]{
      inputs.flAnglePosition, inputs.flVelocity,
      inputs.frAnglePosition, inputs.frVelocity,
      inputs.blAnglePosition, inputs.blVelocity,
      inputs.brAnglePosition, inputs.brVelocity
    });
  }

  /**
   * @param speeds Desired robot-relative chassis speeds
   */
  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    // speeds.discretize(speeds, 0)
    swerveDrive.setChassisSpeeds(speeds);
  }

  /**
   * @return The current chassis speeds of the robot
   */
  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return swerveDrive.getFieldVelocity();
  }

  @Override
  public void setPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    // swerveDrive.setGyroOffset(
    //     new Rotation3d(
    //         swerveDrive.getGyroRotation3d().getX(),
    //         swerveDrive.getGyroRotation3d().getY(),
    //         pose.getRotation().getRadians()));
  }

  /**
   * @param pose Estimated pose as reported by PhotonVision
   * @param timestamp Timestamp of pose reported by PhotonVision
   */
  @Override
  public void addVisionMeasurement(Pose3d pose, double timestamp) {
    var gyroOffset = swerveDrive.getGyroRotation3d();
    swerveDrive.addVisionMeasurement(pose.toPose2d(), timestamp);
    swerveDrive.setGyroOffset(gyroOffset);
    swerveDrive.setGyroOffset(new Rotation3d(0, 0, swerveDrive.getYaw().getRadians()));
    Logger.recordOutput("Photon Pose", pose.toPose2d());
  }
  /**
   * @param module Module number, [0, 3]. 0 -> FL, 1 -> FR, 2 -> BL, 3 -> BR
   * @param motor Motor number, [0, 1]. 0 -> Drive, 1 -> Angle
   * @return The addressed motor
   */
  private CANSparkMax getMotor(int module, int motor) {
    if (motor == 0)
      return ((CANSparkMax) swerveDrive.getModules()[module].getDriveMotor().getMotor());
    else return ((CANSparkMax) swerveDrive.getModules()[module].getAngleMotor().getMotor());
  }

  @Override
  public void setVolts(double left, double right) {
    getMotor(0, 0).setVoltage(left);
    getMotor(2, 0).setVoltage(left);

    getMotor(1, 0).setVoltage(right);
    getMotor(3, 0).setVoltage(right);

  }

}

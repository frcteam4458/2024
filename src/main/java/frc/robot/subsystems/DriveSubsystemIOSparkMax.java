package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.OperatorConstants;
import java.io.File;
import java.io.IOException;

import org.littletonrobotics.junction.Logger;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

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
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(3);
    } catch (IOException e) {
      e.printStackTrace();
    }
    System.out.println(swerveDrive.getSwerveController().config.maxAngularVelocity);

    for (int i = 0; i < 4; i++)
      swerveDrive.getModules()[i].feedforward =
          new SimpleMotorFeedforward(OperatorConstants.kS, OperatorConstants.kV);
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

    inputs.frAnglePosition = swerveDrive.getModules()[1].getAngleMotor().getPosition();
    inputs.frAngleVelocity = swerveDrive.getModules()[1].getAngleMotor().getVelocity();
    inputs.frAngleVolts = getMotor(1, 1).get() * voltage;
    inputs.frAngleAmps = getMotor(1, 1).getOutputCurrent();

    inputs.blAnglePosition = swerveDrive.getModules()[2].getAngleMotor().getPosition();
    inputs.blAngleVelocity = swerveDrive.getModules()[2].getAngleMotor().getVelocity();
    inputs.blAngleVolts = getMotor(2, 1).get() * voltage;
    inputs.blAngleAmps = getMotor(2, 1).getOutputCurrent();

    inputs.brAnglePosition = swerveDrive.getModules()[3].getAngleMotor().getPosition();
    inputs.brAngleVelocity = swerveDrive.getModules()[3].getAngleMotor().getVelocity();
    inputs.brAngleVolts = getMotor(3, 1).get() * voltage;
    inputs.brAngleAmps = getMotor(3, 1).getOutputCurrent();

    inputs.leftEncoderAverage = (inputs.flPosition + inputs.blPosition) / 2;
    inputs.rightEncoderAverage = (inputs.frPosition + inputs.brPosition) / 2;

    inputs.gyroYaw = swerveDrive.getYaw().getDegrees();

    inputs.robotPose = swerveDrive.getPose();
  }

  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerveDrive.setChassisSpeeds(speeds);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return swerveDrive.getFieldVelocity();
  }

  @Override
  public void setPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    swerveDrive.setGyro(
        new Rotation3d(
            swerveDrive.getGyroRotation3d().getX(),
            swerveDrive.getGyroRotation3d().getY(),
            pose.getRotation().getRadians()));
  }

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
}

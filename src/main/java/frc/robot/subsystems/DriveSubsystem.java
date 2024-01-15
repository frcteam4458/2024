// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

  private final DriveSubsystemIO io;
  DriveSubsystemIOInputsAutoLogged inputs = new DriveSubsystemIOInputsAutoLogged();

  Field2d field;
  DifferentialDriveOdometry odometry;

  public static Pose2d robotPose = new Pose2d();

  public DriveSubsystem(DriveSubsystemIO io) {
    this.io = io;
    odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);

    field = new Field2d();

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("DriveSubsystem", inputs);

    Logger.recordOutput("Odometry/Pose", getPose());

    odometry.update(Rotation2d.fromDegrees(getYaw()), leftEncoderAverage(),
    rightEncoderAverage());

    robotPose = getPose();

    if(VisionSubsystem.estimatedPose.isPresent())
      addVisionMeasurement(VisionSubsystem.estimatedPose.get().estimatedPose, VisionSubsystem.estimatedPose.get().timestampSeconds);
  }

  public void addVisionMeasurement(Pose3d pose, double timestamp) {
    io.addVisionMeasurement(pose, timestamp);
  }

  // Drive methods
  public void arcadeDrive(double x, double omega) {
    // var speeds = DifferentialDrive.arcadeDriveIK(y, omega, true);
    // tankDrive(speeds.left, speeds.right);
    var speeds =
        new ChassisSpeeds(
            x * OperatorConstants.kMaxSpeed, 0.0, omega * OperatorConstants.kMaxAngVel);
    driveChassisSpeeds(speeds);
  }

  public void arcadeDrive(double x, double y, double omega) {
    // var speeds = DifferentialDrive.arcadeDriveIK(y, omega, true);
    // tankDrive(speeds.left, speeds.right);
    var speeds =
        new ChassisSpeeds(
            x * OperatorConstants.kMaxSpeed,
            y * OperatorConstants.kMaxSpeed,
            omega * OperatorConstants.kMaxAngVel);
    driveChassisSpeeds(speeds);
  }

  public void arcadeDriveFieldOriented(double x, double y, double omega) {
    var speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x * OperatorConstants.kMaxSpeed,
            y * OperatorConstants.kMaxSpeed,
            omega * OperatorConstants.kMaxAngVel,
            new Rotation2d(Math.toRadians(getYaw())));
    driveChassisSpeeds(speeds);
  }

  public void setWheelSpeeds(double left, double right) {
    driveChassisSpeeds(
        OperatorConstants.kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(left, right)));
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    io.setChassisSpeeds(speeds);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return io.getChassisSpeeds();
  }

  // Getters / Setters

  public void setPose(double x, double y, double yaw) {
    setPose(new Pose2d(x, y, new Rotation2d(Math.toRadians(yaw))));
  }

  public void setPose(Pose2d pose) {
    // odometry.resetPosition(pose.getRotation(), pose.getX(), pose.getY(), pose);
    io.setPose(pose);
    Logger.recordOutput("setPose", pose);
  }

  public void resetPose() {
    setPose(0, 0, 0);
  }

  public Pose2d getPose() {
    return inputs.robotPose;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    var speeds =
        new DifferentialDriveWheelSpeeds(
            (inputs.flVelocity + inputs.brVelocity) / 2,
            (inputs.frVelocity + inputs.brVelocity) / 2);

    return speeds;
  }

  public double getYaw() {
    return inputs.gyroYaw;
  }

  public double leftEncoderAverage() {
    return inputs.leftEncoderAverage;
  }

  public double rightEncoderAverage() {
    return inputs.rightEncoderAverage;
  }

  // public void tankDrive(double left, double right) {
  //   tankDriveVolts(left * RobotController.getBatteryVoltage(), right *
  // RobotController.getBatteryVoltage());
  // }

  // public void tankDriveVolts(double leftV, double rightV) {
  //   io.setLeftVoltage(leftV);
  //   io.setRightVoltage(rightV);
  // }

}

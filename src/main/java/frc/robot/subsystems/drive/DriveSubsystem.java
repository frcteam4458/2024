// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

  private final DriveSubsystemIO io;
  DriveSubsystemIOInputsAutoLogged inputs = new DriveSubsystemIOInputsAutoLogged();

  Field2d field;


  public SysIdRoutine sysIdRoutine;

  public static Pose2d robotPose = new Pose2d();

  public DriveSubsystem(DriveSubsystemIO io) {
    this.io = io;
    field = new Field2d();

    // Characterization object
    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null, null, null,
        (state) -> Logger.recordOutput("SysIdTestState", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          setVolts(volts.in(Volts), volts.in(Volts));
        },
        log -> {
          log.motor("drive-left").voltage(Volts.of((inputs.flVolts + inputs.blVolts) / 2.0))
            .linearPosition(Meters.of(leftEncoderAverage()))
            .linearVelocity(MetersPerSecond.of((inputs.flVelocity + inputs.blVelocity) / 2.0));

          log.motor("drive-right").voltage(Volts.of((inputs.frVolts + inputs.brVolts) / 2.0))
            .linearPosition(Meters.of(rightEncoderAverage()))
            .linearVelocity(MetersPerSecond.of((inputs.frVelocity + inputs.brVelocity) / 2.0));
        }, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("DriveSubsystem", inputs);

    Logger.recordOutput("Odometry/Pose", getPose());

    robotPose = getPose();

    // Perhaps its a good idea to enable pose estimation from a command rather than statically accessing the vision subsystem
    if(VisionSubsystem.estimatedPoseBack.isPresent() && Robot.isReal())
      addVisionMeasurement(VisionSubsystem.estimatedPoseBack.get().estimatedPose, VisionSubsystem.estimatedPoseBack.get().timestampSeconds);

    if(VisionSubsystem.estimatedPoseFront.isPresent() && Robot.isReal())
      addVisionMeasurement(VisionSubsystem.estimatedPoseFront.get().estimatedPose, VisionSubsystem.estimatedPoseFront.get().timestampSeconds);
  }

  public void addVisionMeasurement(Pose3d pose, double timestamp) {
    io.addVisionMeasurement(pose, timestamp);
  }

  // Drive methods
  public void arcadeDrive(double x, double omega) {
 
    var speeds =
        new ChassisSpeeds(
            x * HardwareConstants.kMaxSpeed, 0.0, omega * HardwareConstants.kMaxAngVel);
    driveChassisSpeeds(speeds);
  }

  public void arcadeDrive(double x, double y, double omega) {
    var speeds =
        new ChassisSpeeds(
            x * HardwareConstants.kMaxSpeed,
            y * HardwareConstants.kMaxSpeed,
            omega * HardwareConstants.kMaxAngVel);
    drive(speeds);
  }

  public void arcadeDriveFieldOriented(double x, double y, double omega) {
    if(RobotContainer.isRed()) {
      x = -x;
      y = -y;
    }
    var speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x * HardwareConstants.kMaxSpeed,
            y * HardwareConstants.kMaxSpeed,
            omega * HardwareConstants.kMaxAngVel,
            getPose().getRotation());
    drive(speeds);
  }

  public void setWheelSpeeds(double left, double right) {
    driveChassisSpeeds(
        OperatorConstants.kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(left, right)));
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    io.setChassisSpeeds(speeds);
  }

  public void drive(ChassisSpeeds speeds) {
    io.drive(speeds);
  }

  public void driveFieldOriented(ChassisSpeeds speeds) {
    io.driveFieldOriented(speeds);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return io.getChassisSpeeds();
  }

  // Getters / Setters

  public void setPose(double x, double y, double yaw) {
    setPose(new Pose2d(x, y, new Rotation2d(Math.toRadians(yaw))));
  }

  public void setPose(Pose2d pose) {
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
            (inputs.flVelocity + inputs.brVelocity) / 2.0,
            (inputs.frVelocity + inputs.brVelocity) / 2.0);

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

  /**
   * Ideally should only be used for system identification
   * @param left Left side voltage
   * @param right Right side voltage
   */
  public void setVolts(double left, double right) {
    io.setVolts(left, right);
  }

  public void setCoast(boolean coast) {
    io.setCoast(coast);
  }
}

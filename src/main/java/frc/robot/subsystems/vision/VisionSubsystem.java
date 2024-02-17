// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class VisionSubsystem extends SubsystemBase {

  PhotonCamera backCamera;
  PhotonCamera frontCamera;
  PhotonCameraSim cameraSim;
  PhotonPoseEstimator poseEstimator;
  PhotonPipelineResult result;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget bestTarget;

  VisionSystemSim sim;

  public static Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
  public static double fpgaTimestamp = 0.0;

  Transform3d robotToCamera;

  public VisionSubsystem() {
    // -12.75 , 17.25

    robotToCamera = new Transform3d(Units.inchesToMeters(-12.75),
      0, Units.inchesToMeters(17.25), new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(180)));
    backCamera = new PhotonCamera("back");
    
    // SimCameraProperties properties = new SimCameraProperties();
    // properties.setCalibration(800, 600, Rotation2d.fromDegrees(75));
    // cameraSim = new PhotonCameraSim(backCamera, properties);
    // if(Robot.isSimulation()) cameraSim.enableDrawWireframe(true);

    // sim = new VisionSystemSim("main");
    // sim.addCamera(
    //   cameraSim,
    //   robotToCamera);


    try {
      poseEstimator =
          new PhotonPoseEstimator(
              AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              backCamera,
              robotToCamera);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

      sim.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    result = backCamera.getLatestResult();

    if(result.hasTargets()) {
      targets = result.getTargets();
      bestTarget = result.getBestTarget();
    }

    poseEstimator.setReferencePose(DriveSubsystem.robotPose);
    estimatedPose = poseEstimator.update();

    if(estimatedPose.isPresent()) {
      Logger.recordOutput("Vision/EstimatedPose", estimatedPose.get().estimatedPose.toPose2d());
    }
    fpgaTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void simulationPeriodic() {
    // sim.update(DriveSubsystem.robotPose);
  }

  public List<PhotonTrackedTarget> getTargets() {
    return targets;
  }

  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
  }

  public boolean hasTargets() {
    return result.hasTargets();
  }
}

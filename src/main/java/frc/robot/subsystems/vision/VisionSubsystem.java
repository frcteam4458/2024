// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

  PhotonCamera frontCamera;
  PhotonCamera backCamera;

  PhotonCameraSim frontCameraSim;
  PhotonCameraSim backCameraSim;

  PhotonPoseEstimator frontPoseEstimator;
  PhotonPoseEstimator backPoseEstimator;

  PhotonPipelineResult result;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget bestTarget;

  VisionSystemSim sim;

  public static Optional<EstimatedRobotPose> estimatedPoseFront = Optional.empty();
  public static Optional<EstimatedRobotPose> estimatedPoseBack = Optional.empty();

  Transform3d robotToFrontCamera;
  Transform3d robotToBackCamera;

  public VisionSubsystem() {

    robotToFrontCamera = new Transform3d(Units.inchesToMeters(-9.5),
      0, Units.inchesToMeters(17.25), new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(0)));

    robotToBackCamera = new Transform3d(Units.inchesToMeters(-10.915),
      0, Units.inchesToMeters(17.25), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)));

    frontCamera = new PhotonCamera("front");
    backCamera = new PhotonCamera("back");

    if(Robot.isSimulation()) {
      sim = new VisionSystemSim("main");

      SimCameraProperties properties = new SimCameraProperties();
      properties.setCalibration(800, 600, Rotation2d.fromDegrees(70));

      frontCameraSim = new PhotonCameraSim(frontCamera, properties);
      backCameraSim = new PhotonCameraSim(backCamera, properties);
  
      
      frontCameraSim.enableDrawWireframe(true);
      backCameraSim.enableDrawWireframe(true);


      sim.addCamera(frontCameraSim, robotToFrontCamera);
      sim.addCamera(backCameraSim, robotToBackCamera);
    }

    try {
      frontPoseEstimator = new PhotonPoseEstimator(
              AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              frontCamera,
              robotToFrontCamera);
      frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      backPoseEstimator =
          new PhotonPoseEstimator(
              AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              backCamera,
              robotToBackCamera);
      backPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      sim.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    estimatedPoseFront = frontPoseEstimator.update();
    estimatedPoseBack = backPoseEstimator.update();
    if(estimatedPoseFront.isPresent()) {
      Logger.recordOutput("Vision/EstimatedPoseFront", estimatedPoseFront.get().estimatedPose.toPose2d());
    }

    if(estimatedPoseBack.isPresent()) {
      Logger.recordOutput("Vision/EstimatedPoseBack", estimatedPoseBack.get().estimatedPose.toPose2d());
    }

  }

  @Override
  public void simulationPeriodic() {
    sim.update(DriveSubsystem.robotPose);
  }

  public List<PhotonTrackedTarget> getTargets() {
    List<PhotonTrackedTarget> list = frontCamera.getLatestResult().getTargets();
    list.addAll(backCamera.getLatestResult().getTargets());
    return list;
  }

  public PhotonTrackedTarget getBestTarget() {
    return null;
  }

  public boolean hasTargets() {
    return backCamera.getLatestResult().hasTargets() || frontCamera.getLatestResult().hasTargets();
  }

  public Optional<PhotonTrackedTarget> getTag(int id) {
    for(PhotonTrackedTarget target : getTargets()) {
      if(target.getFiducialId() == id) {
        return Optional.of(target);
      }
    }
    return Optional.empty();
  }

  public double distanceToTarget(PhotonTrackedTarget target) {
    return Math.sqrt(Math.pow(target.getBestCameraToTarget().getX(), 2) + Math.pow(target.getBestCameraToTarget().getY(), 2)); 
  }
}

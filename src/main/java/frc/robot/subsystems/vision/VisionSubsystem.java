// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;
import java.util.Optional;

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

  PhotonCamera camera;
  PhotonCameraSim cameraSim;
  PhotonPoseEstimator poseEstimator;
  PhotonPipelineResult result;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget bestTarget;

  VisionSystemSim sim;

  public static Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
  public static double fpgaTimestamp = 0.0;

  public VisionSubsystem() {
    camera = new PhotonCamera("camera");
    SimCameraProperties properties = new SimCameraProperties();
    // properties.setCalibration(640, 480, Rotation2d.fromDegrees(70));
    cameraSim = new PhotonCameraSim(camera, properties);
    if(Robot.isSimulation()) cameraSim.enableDrawWireframe(true);

    sim = new VisionSystemSim("main");
    sim.addCamera(cameraSim, new Transform3d(0, 0, 0, new Rotation3d()));
    try {
      poseEstimator =
          new PhotonPoseEstimator(
              AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              camera,
              new Transform3d());
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

      sim.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();

    if(result.hasTargets()) {
      targets = result.getTargets();
      bestTarget = result.getBestTarget();
    }
    poseEstimator.setReferencePose(DriveSubsystem.robotPose);
    estimatedPose = poseEstimator.update();
    // fpgaTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void simulationPeriodic() {
    sim.update(DriveSubsystem.robotPose);
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

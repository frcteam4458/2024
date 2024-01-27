// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public class SpeakerAlignCommand extends AlignCommand {
    
    public SpeakerAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier flip,
            double ap, double ai, double ad) {
        super(driveSubsystem, visionSubsystem, flip, ap, ai, ad);

    }
    
    @Override
    public Rotation2d getAngle(DriveSubsystem driveSubsystem, BooleanSupplier flip) {
        var targetOptional = getTarget(7);
        if(targetOptional.isPresent()) {
            var target = targetOptional.get();
            double yaw = target.getYaw();
            yaw = Math.toRadians(yaw);
            var rotation = Rotation2d.fromRadians(driveSubsystem.getPose().getRotation().getRadians() - (yaw));
            return rotation;
        }
        return new Rotation2d();
    }

    public Optional<PhotonTrackedTarget> getTarget(int targetid) {
        if(visionSubsystem.hasTargets()) {
            for(int i = 0; i < visionSubsystem.getTargets().size(); i++) {
                int id = visionSubsystem.getTargets().get(i).getFiducialId();
                if(id == targetId) {
                    return Optional.of(visionSubsystem.getTargets().get(i));
                }
            }
        }

        return Optional.empty();
    }

    @Override
    public void execute() {
        profiledYawController.setGoal(getAngle(driveSubsystem, flip).getRadians());
        super.execute();
    }
}

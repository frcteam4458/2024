// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public class NoteLock extends AlignCommand {

    public NoteLock(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier flip, double ap,
            double ai, double ad) {
        super(driveSubsystem, visionSubsystem, flip, ap, ai, ad);
    }

    @Override
    public Rotation2d getAngle(DriveSubsystem driveSubsystem, BooleanSupplier flip) {
        if(visionSubsystem.getBestTarget().isPresent()) {
            var note = visionSubsystem.getBestTarget().get();

            Logger.recordOutput("NoteLock/Note Yaw", note.getYaw());
        }

        return driveSubsystem.getPose().getRotation();
    }

    @Override
    public double getOmega() {
        return 0;
    }
    
}

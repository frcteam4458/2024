// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public class SpeakerAlignCommand extends AlignCommand {

    public SpeakerAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier flip,
            double ap, double ai, double ad) {
        super(driveSubsystem, visionSubsystem, flip, ap, ai, ad);

    }
    
    @Override
    public Rotation2d getAngle(DriveSubsystem driveSubsystem, BooleanSupplier flip) {
        return Rotation2d.fromDegrees(20);
    }
}

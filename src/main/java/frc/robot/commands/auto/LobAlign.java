// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public class LobAlign extends AlignCommand {

    public LobAlign(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier flip, double ap,
            double ai, double ad) {
        super(driveSubsystem, visionSubsystem, flip, 0.3, ai, 0.03);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public Rotation2d getAngle(DriveSubsystem driveSubsystem, BooleanSupplier flip) {
        var ampTranslation = PositionConstants.kAmpPose.getTranslation();
        var robotTranslation = driveSubsystem.getPose().getTranslation();
        if(flip.getAsBoolean()) ampTranslation = GeometryUtil.flipFieldPosition(ampTranslation);

        double theta = Math.atan2(robotTranslation.getY() - ampTranslation.getY(), robotTranslation.getX() - ampTranslation.getX());

        Logger.recordOutput("LobAlign/Theta Out", theta);
        return Rotation2d.fromRadians(theta);
    }
    
}

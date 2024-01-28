// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// TODO: make this into a more generic object
public class SpeakerAlign extends TeleopCommand {
    PIDController yawController;
    public ProfiledPIDController profiledYawController;
    BooleanSupplier flip;
    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;

    int targetId = 7;

    public SpeakerAlign(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier flip, double ap, double ai, double ad) {
        super(driveSubsystem);
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        yawController = new PIDController(ap, ai, ad);
        profiledYawController = new ProfiledPIDController(ap, ai, ad, new TrapezoidProfile.Constraints(1, 0.25));
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        profiledYawController.enableContinuousInput(-Math.PI, Math.PI);
        this.flip = flip;
    }

    public Rotation2d getAngle(DriveSubsystem driveSubsystem, BooleanSupplier flip) {
        Translation2d translation = PositionConstants.kSpeakerPosition;
        boolean flipPosition = flip.getAsBoolean();

        if(flipPosition) {
            translation = GeometryUtil.flipFieldPosition(translation);
        }

        var rotation = Rotation2d.fromRadians(
            Math.atan((driveSubsystem.getPose().getY() - translation.getY()) /
            (driveSubsystem.getPose().getX() - translation.getX()))
        );

        if(flip.getAsBoolean()) {
            return rotation;
        } else {
            return Rotation2d.fromRadians((rotation.getRadians()));
        }
    }

    @Override
    public void execute() {
        yawController.setSetpoint(getAngle(driveSubsystem, flip).getRadians());
        super.execute(); 
    }

    @Override
    public double getOmega() {
        return yawController.calculate(driveSubsystem.getPose().getRotation().getRadians());
    }
    
}

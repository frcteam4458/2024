// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
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

        Logger.recordOutput("X Diff", (driveSubsystem.getPose().getX() - translation.getX()));
        Logger.recordOutput("Y Diff", (driveSubsystem.getPose().getY() - translation.getY()));
        var rotation = Rotation2d.fromRadians(
            Math.atan((driveSubsystem.getPose().getY() - translation.getY()) /
            (driveSubsystem.getPose().getX() - translation.getX()))
        );

        if(flip.getAsBoolean()) {
            return rotation;
        } else {
            return Rotation2d.fromRadians(Math.PI + rotation.getRadians());
        }
    }

    @Override
    public void execute() {
        yawController.setSetpoint(getAngle(driveSubsystem, flip).getRadians());
        Logger.recordOutput("Yaw Diff", getAngle(driveSubsystem, flip));
        super.execute(); 
    }

    @Override
    public double getOmega() {
        return yawController.calculate(driveSubsystem.getPose().getRotation().getRadians());
    }
    
}

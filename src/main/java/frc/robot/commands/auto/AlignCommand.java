// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public class AlignCommand extends TeleopCommand {
    PIDController yawController;
    public ProfiledPIDController profiledYawController;
    BooleanSupplier flip;
    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;

    int targetId = 7;

    public AlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier flip, double ap, double ai, double ad) {
        super(driveSubsystem);
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        yawController = new PIDController(ap, ai, ad);
        profiledYawController = new ProfiledPIDController(ap, ai, ad, new TrapezoidProfile.Constraints(0.05, 0.01));
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        profiledYawController.enableContinuousInput(-Math.PI, Math.PI);
        this.flip = flip;
    }

    public Rotation2d getAngle(DriveSubsystem driveSubsystem, BooleanSupplier flip) {
        Translation2d translation = PositionConstants.kSpeakerPosition;
        boolean flipPosition = flip.getAsBoolean();

        if(flipPosition) {
            // translation = GeometryUtil.flipFieldPosition(translation);
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
        double output = -yawController.calculate(driveSubsystem.getPose().getRotation().getRadians());;
        if(output < -0.25) output = -0.25;
        if(0.25 < output) output = 0.25;
        Logger.recordOutput("Align Output", output);
        return output;
    }
    
}

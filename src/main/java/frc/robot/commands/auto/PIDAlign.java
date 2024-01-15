// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class PIDAlign extends Command {
    
    DriveSubsystem driveSubsystem;
    PIDController xController;
    PIDController yController;
    PIDController yawController;

    Pose2d targetPose;
    BooleanSupplier flipPose;
    boolean poseFlipped = false;
    
    public PIDAlign(DriveSubsystem driveSubsystem, Pose2d targetPose, BooleanSupplier flipPose, double p, double i, double d, double ap, double ai, double ad) {
        this.driveSubsystem = driveSubsystem;
        xController = new PIDController(p, i, d);
        yController = new PIDController(p, i, d);
        yawController = new PIDController(ap, ai, ad);
        this.targetPose = targetPose;
        this.flipPose = flipPose;
        yawController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        if(!poseFlipped) { // pose not flipped
            if(flipPose.getAsBoolean()) { // we should flip the pose
                targetPose = GeometryUtil.flipFieldPose(targetPose); // flip
                poseFlipped = true; // mark it flipped
            }
        } else { // pose is flipped
            if(!flipPose.getAsBoolean()) { // shouldnt be flipped but it is
                targetPose = GeometryUtil.flipFieldPose(targetPose); // flip
                poseFlipped = false; // mark it unflipped
            }
        }
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        yawController.setSetpoint(targetPose.getRotation().getRadians());
        driveSubsystem.arcadeDrive(0, 0, 0);
    }

    @Override
    public void execute() {
        Pose2d pose = driveSubsystem.getPose();
        double x = MathUtil.clamp(xController.calculate(pose.getX()), -3.0, 3.0);
        double y = MathUtil.clamp(yController.calculate(pose.getY()), -3.0, 3.0);
        double yaw = MathUtil.clamp(yawController.calculate(pose.getRotation().getRadians()), -3.0, 3.0);
        driveSubsystem.driveChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, yaw, pose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.arcadeDrive(0, 0, 0);
    }
}

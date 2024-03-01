// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TrapAlign extends Command {
    
    DriveSubsystem driveSubsystem;
    PIDController xController;
    PIDController yController;
    PIDController yawController;

    Pose2d targetPose;
    BooleanSupplier flipPose;
    boolean poseFlipped = false;
    
    public TrapAlign(DriveSubsystem driveSubsystem, Pose2d targetPose, BooleanSupplier flipPose, double p, double i, double d, double ap, double ai, double ad) {
        this.driveSubsystem = driveSubsystem;
        xController = new PIDController(p, i, d);
        yController = new PIDController(p, i, d);
        yawController = new PIDController(ap, ai, ad);
        // this.targetPose = targetPose;
        this.flipPose = flipPose;
        yawController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d pose = getClosestPose();
        System.out.println("Closest Pose Y: " + pose.getY());
        xController.setSetpoint(pose.getX());
        yController.setSetpoint(pose.getY());
        yawController.setSetpoint(pose.getRotation().getRadians());
        double x = MathUtil.clamp(xController.calculate(pose.getX()), -3.0, 3.0);
        double y = yController.calculate(pose.getY());
        double yaw = MathUtil.clamp(yawController.calculate(pose.getRotation().getRadians()), -3.0, 3.0);
        System.out.println("Y Out: " + y);
        driveSubsystem.driveChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, yaw, driveSubsystem.getPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.arcadeDrive(0, 0, 0);
    }

    public Pose2d getClosestPose() {
        Pose2d ampTrap = getFlipped(PositionConstants.kAmpTrap);
        Pose2d sourceTrap = getFlipped(PositionConstants.kSourceTrap);
        Pose2d centerTrap = getFlipped(PositionConstants.kCenterTrap);

        // im jetlagged the hell out so i msorry for this code
        double[] distances = new double[] {
            ampTrap.getTranslation().getDistance(driveSubsystem.getPose().getTranslation()),
            sourceTrap.getTranslation().getDistance(driveSubsystem.getPose().getTranslation()),
            centerTrap.getTranslation().getDistance(driveSubsystem.getPose().getTranslation())
        };

        int minIndex = 0;
        double minVal = 100;
        for(int i = 0; i < 3; i++) {
            if(distances[i] < minVal) {
                minVal = distances[i];
                minIndex = i;
            }
        }

        switch(minIndex) {
            case 0:
                return ampTrap;
            case 1:
                return sourceTrap;
            default:
                return centerTrap;
        }
    }

    public Pose2d getFlipped(Pose2d pose) {
        if(flipPose.getAsBoolean()) return GeometryUtil.flipFieldPose(pose);
        return pose;
    }
}

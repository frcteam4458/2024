// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.MathUtil;
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
public class AmpAlign extends TeleopCommand {
    PIDController xController;
    PIDController yawController;
    
    BooleanSupplier flip;
    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;

    int targetId = 7;

    public AmpAlign(DriveSubsystem driveSubsystem, BooleanSupplier flip, double p, double i, double d, double ap, double ai, double ad) {
        super(driveSubsystem);
        this.driveSubsystem = driveSubsystem;
        xController = new PIDController(p, i, d);
        yawController = new PIDController(ap, ai, ad);
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        yawController.setSetpoint(-Math.PI / 2.0);
        this.flip = flip;
    }

    @Override   
    public void execute() {
        var translation = flip.getAsBoolean() ? GeometryUtil.flipFieldPosition(PositionConstants.kAmpPose.getTranslation()) : PositionConstants.kAmpPose.getTranslation();
        Logger.recordOutput("AmpAlign/TargetX", translation.getX());
        xController.setSetpoint(translation.getX());
        super.execute();
    }

    @Override
    public double getY() {
        double output = xController.calculate(driveSubsystem.getPose().getX());
        if(output < -0.1) output = -0.1;
        if(0.1 < output) output = 0.1;
        Logger.recordOutput("AmpAlign/PoseX", driveSubsystem.getPose().getX());
        Logger.recordOutput("AmpAlign/OutputX", output);
        return output;
    }

    @Override
    public double getOmega() {
        double output = yawController.calculate(driveSubsystem.getPose().getRotation().getRadians());
        if(output < -0.1) output = -0.1;
        if(0.1 < output) output = 0.1;
        Logger.recordOutput("Align Output", output);
        return output;
    }
    
}

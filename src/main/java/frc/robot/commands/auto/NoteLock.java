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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public class NoteLock extends TeleopCommand {
    PIDController yawController;
    BooleanSupplier flip;
    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;

    int targetId = 7;

    Rotation2d angle;

    public NoteLock(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier flip, double ap, double ai, double ad) {
        super(driveSubsystem);
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        yawController = new PIDController(0.1, ai, 0.0);
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        this.flip = flip;
    }


    public Rotation2d getAngle(DriveSubsystem driveSubsystem, BooleanSupplier flip) {
        if(visionSubsystem.getBestTarget().isPresent()) {
            Logger.recordOutput("Note Lock/Note Yaw", visionSubsystem.getBestTarget().get().getYaw());
            return Rotation2d.fromDegrees(driveSubsystem.getPose().getRotation().getDegrees() - visionSubsystem.getBestTarget().get().getYaw());
        } else {
            return driveSubsystem.getPose().getRotation();
        }
    }

    @Override   
    public void execute() {
        yawController.setSetpoint(MathUtil.angleModulus(getAngle(driveSubsystem, flip).getRadians()));
        Logger.recordOutput("Note Lock/Yaw Diff", getAngle(driveSubsystem, flip));
        super.execute();
    }

    @Override
    public double getOmega() {
        if(!visionSubsystem.getBestTarget().isPresent()) {
            return super.getOmega();
        }
        double output = yawController.calculate(driveSubsystem.getPose().getRotation().getRadians());
        if(output < -0.4) output = -0.4;
        if(0.4 < output) output = 0.4;
        Logger.recordOutput("Note Lock/Align Output", output);
        // output = 0;
        return output;
    }

    
    
}

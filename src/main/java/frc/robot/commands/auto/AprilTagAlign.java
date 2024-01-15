// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public class AprilTagAlign extends TeleopCommand {
    
    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    PIDController yawController;
    int targetId;

    double angle = 0;

    public AprilTagAlign(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, int id, double ap, double ai, double ad) {
        super(driveSubsystem);
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        yawController = new PIDController(ap, ai, ad);
        this.targetId = id;
        SmartDashboard.putData(yawController);
    }

    @Override
    public void initialize() {
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        

    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public double getOmega() {
        // Optional<PhotonTrackedTarget> targetOptional = getTarget();
        // if(targetOptional.isPresent()) {
        //     PhotonTrackedTarget target = targetOptional.get();

            var target = getTarget();
            if(target.isPresent()) {
                angle = Math.toRadians(target.get().getYaw());
                yawController.setSetpoint(driveSubsystem.getPose().getRotation().getRadians() + angle);
            }
            double output = yawController.calculate(driveSubsystem.getPose().getRotation().getRadians());
            // Logger.recordOutput("Target Yaw", target.getYaw());
            Logger.recordOutput("April PID Output", output);
            Logger.recordOutput("April PID Setpoint", yawController.getSetpoint());
            Logger.recordOutput("April PID Error", yawController.getPositionError());
            // if(output > 1) output = 1;
            // if(output < -1) output = -1;
            return output;
        // }
        // return 0;
    }

    public Optional<PhotonTrackedTarget> getTarget() {
        if(visionSubsystem.hasTargets()) {
            for(int i = 0; i < visionSubsystem.getTargets().size(); i++) {
                int id = visionSubsystem.getTargets().get(i).getFiducialId();
                if(id == targetId) {
                    return Optional.of(visionSubsystem.getTargets().get(i));
                }
            }
        }

        return Optional.empty();
    }
}

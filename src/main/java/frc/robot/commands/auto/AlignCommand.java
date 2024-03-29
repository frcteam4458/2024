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
        yawController = new PIDController(0.3, ai, 0.0); // 0.5 0.025
        profiledYawController = new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(4, 1));
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        profiledYawController.enableContinuousInput(-Math.PI, Math.PI);
        this.flip = flip;
        yawController.setTolerance(Math.toRadians(5), 1);
        // SmartDashboard.putNumber("KP", 0.5);
        // SmartDashboard.putNumber("KI", 0.0);
        // SmartDashboard.putNumber("KD", 0.0);
        // SmartDashboard.putNumber("MaxVel", 0.0);
        // SmartDashboard.putNumber("MaxAccel", 0.0);
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
            Math.atan2((driveSubsystem.getPose().getY() - translation.getY()),
            (driveSubsystem.getPose().getX() - translation.getX()))
        );

        double yVel = driveSubsystem.getChassisSpeeds().vyMetersPerSecond;

        Rotation2d turn = Rotation2d.fromDegrees(yVel*10);
        turn = Rotation2d.fromDegrees(0);
        if(turn.getDegrees() < -5) turn = Rotation2d.fromDegrees(-5);
        if(5 < turn.getDegrees()) turn = Rotation2d.fromDegrees(5);
        rotation = rotation.plus(turn);

        if(flip.getAsBoolean()) {
            return rotation;
        } else {
            return Rotation2d.fromRadians(rotation.getRadians());
        }
    }

    public boolean atSetpoint() {
        return yawController.atSetpoint();
    }

    @Override   
    public void execute() {

        // yawController.setP(SmartDashboard.getNumber("KP", 0.5));
        // yawController.setI(SmartDashboard.getNumber("KI", 0.0));
        // yawController.setD(SmartDashboard.getNumber("KD", 0.0));
        // profiledYawController.setConstraints(
        //     new TrapezoidProfile.Constraints(SmartDashboard.getNumber("MaxVel", 0.0),
        //     SmartDashboard.getNumber("MaxAccel", 0.0)));

        yawController.setSetpoint(MathUtil.angleModulus(getAngle(driveSubsystem, flip).getRadians()));
        profiledYawController.setGoal(MathUtil.angleModulus(getAngle(driveSubsystem, flip).getRadians()));
        Logger.recordOutput("Yaw Diff", getAngle(driveSubsystem, flip));
        super.execute();
    }

    @Override
    public double getOmega() {
        double output = yawController.calculate(driveSubsystem.getPose().getRotation().getRadians());;
        // output = prof    iledYawController.calculate(driveSubsystem.getPose().getRotation().getRadians());;
        if(output < -0.25) output = -0.25;
        if(0.25 < output) output = 0.25;
        Logger.recordOutput("Align Output", output);
        return output;
    }

    @Override
    public void end(boolean interrupted) {
        
        super.end(interrupted);
    }
    
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControlConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Add your docs here. */
public class HeadingCommand extends TeleopCommand {

    PIDController pidController;
    double angle;

    public HeadingCommand(DriveSubsystem driveSubsystem, double angle) {
        super(driveSubsystem);
        this.angle = angle;

        pidController = new PIDController(ControlConstants.kAP, ControlConstants.kAI, 0);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        // pidController.setTolerance(Math.toRadians(5));

    }

    @Override
    public void execute() {
        super.execute();

        if(Math.abs(super.genericController.getRawAxis(4)) > 0.01) {
            this.cancel();
        }

        if(RobotContainer.isRed()) pidController.setSetpoint(MathUtil.angleModulus(Math.toRadians(angle) + Math.PI));
        pidController.setSetpoint(MathUtil.angleModulus(Math.toRadians(angle)));
    }

    @Override
    public double getOmega() {
        double output = pidController.calculate(driveSubsystem.getPose().getRotation().getRadians());
        if(output < -0.25) output = -0.25;
        if(0.25 < output) output = 0.25;
        Logger.recordOutput("HeadingCommand/Setpoint", pidController.getSetpoint());
        Logger.recordOutput("HeadingCommand/Position", driveSubsystem.getPose().getRotation().getRadians());
        Logger.recordOutput("HeadingCommand/Output", output);
        return output;
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(super.genericController.getRawAxis(4)) > 0.01) || pidController.atSetpoint();
    }
    
}

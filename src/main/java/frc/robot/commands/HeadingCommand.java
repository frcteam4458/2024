// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

        pidController = new PIDController(ControlConstants.kAP, 0, 0);
        pidController.enableContinuousInput(-180, 180);

        if(RobotContainer.isRed()) {
            angle += 180;
        }
    }

    @Override
    public void execute() {
        super.execute();

        if(Math.abs(super.genericController.getRawAxis(4)) < 0.01) {
            this.cancel();
        }
    }

    @Override
    public double getOmega() {
        double output = pidController.calculate(driveSubsystem.getPose().getRotation().getDegrees());
        if(Math.abs(output) < -1) output = 1;
        if(1 < Math.abs(output)) output = 1;
        return output;
    }
    
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;

/** Add your docs here. */
public class FlywheelCommand extends Command {
    Flywheel flywheel;
    double rpm;

    public FlywheelCommand(Flywheel flywheel, double rpm) {
        this.flywheel = flywheel;
        this.rpm = rpm;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.setRPM(rpm);
        Logger.recordOutput("Flywheel/Flywheel Command", true);
    }

    @Override
    public boolean isFinished() {
        Logger.recordOutput("Flywheel/Flywheel Command", false);
        return flywheel.atSetpoint();
    }

}

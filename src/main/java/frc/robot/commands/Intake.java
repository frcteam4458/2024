// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControlConstants;
import frc.robot.subsystems.feeder.Feeder;

/** Add your docs here. */
public class Intake extends Command {
    Feeder feeder;

    boolean detectedTop = false;
    boolean detectedBottom = false;
    boolean setFinalPosition = false;
    double finalPosition = 0.0;
    boolean running = false;
    
    public Intake(Feeder feeder) {
        this.feeder = feeder;
        
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        detectedTop = false;
        detectedBottom = false;
        setFinalPosition = false;
        finalPosition = 0.0;
        running = true;
        Logger.recordOutput("Intake/Running", running);
    }

    @Override
    public void execute() {
        // if(feeder.getBottom()) {
        //     detectedBottom = true;
        // }
        // if(feeder.getTop() && !detectedBottom) {
        //     detectedTop = true;
        // }

        if(feeder.getTop()) {
            detectedTop = true;
        }

        if(!setFinalPosition) {
            // if(detectedBottom) {
            //     // finalPosition = feeder.getPosition() + ControlConstants.kBottomStopRotations;
            //     setFinalPosition = true;
            // }

            if(detectedTop) {
                finalPosition = feeder.getPosition();//+ ControlConstants.kTopStopRotations;
                setFinalPosition = true;
            }
        }

        Logger.recordOutput("Intake/detectedBottom", detectedBottom);
        Logger.recordOutput("Intake/detectedTop", detectedTop);
        Logger.recordOutput("Intake/finalPosition", finalPosition);
        Logger.recordOutput("Intake/setFinalPosition", setFinalPosition);

        if(detectedBottom || detectedTop)
            feeder.setSetpoint(finalPosition);
        else
            feeder.set(ControlConstants.kFeederMagnitude);
    }

    @Override
    public boolean isFinished() {
        return setFinalPosition;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        running = false;
        Logger.recordOutput("Intake/Running", running);
    }
}

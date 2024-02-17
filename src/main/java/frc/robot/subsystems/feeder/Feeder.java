// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlConstants;

/** Add your docs here. */
public class Feeder extends SubsystemBase {

    FeederIO io;
    FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
    PIDController feederController;
    boolean pidControl = false;

    GenericHID driver = new GenericHID(0);

    public Feeder(FeederIO io) {
        this.io = io;
        feederController = new PIDController(ControlConstants.kFeederP, 0, 0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
        Logger.recordOutput("Feeder/Setpoint", feederController.getSetpoint());
        
        if(pidControl) {
            double output = feederController.calculate(getPosition());
            // if((12 * ControlConstants.kFeederMagnitude) < output) {
            //     output = 12 * ControlConstants.kFeederMagnitude;
            // }
            if(!driver.getRawButton(4))
                setVoltage(output);
            else
                setVoltage(0);
        }

        if(driver.getRawButton(4)) {
            setVoltage(0);
        }
    }

    public void set(double value) {
        if(!driver.getRawButton(4))
            io.set(value);
        pidControl = false;
    }

    public void setVoltage(double volts) {
        if(!driver.getRawButton(4))
            io.setVoltage(volts);
    }
    
    public void setSetpoint(double setpoint) {
        feederController.setSetpoint(setpoint);
        pidControl = true;
    }

    public double getPosition() {
        return inputs.position;
    }

    public boolean getTop() {
        return inputs.topSensor;
    }

    public boolean getBottom() {
        return inputs.bottomSensor;
    }

    public boolean atSetpoint() {
        return feederController.atSetpoint();
    }

    
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ControlConstants;

/** Add your docs here. */
public class Flywheel extends SubsystemBase {
    
    FlywheelIO io;
    FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    double setpoint;

    PIDController topController;
    PIDController bottomController;

    SimpleMotorFeedforward feedforward;

    GenericHID driver;

    double rpm = 0;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        topController = new PIDController(
            ControlConstants.kFlywheelP,
            ControlConstants.kFlywheelI,
            ControlConstants.kFlywheelD);
        bottomController = new PIDController(
            ControlConstants.kFlywheelP,
            ControlConstants.kFlywheelI,
            ControlConstants.kFlywheelD);

        feedforward = new SimpleMotorFeedforward(0, ControlConstants.kFlywheelV);

        driver = new GenericHID(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        Logger.recordOutput("Flywheel/Top Setpoint Reached", topController.atSetpoint());
        Logger.recordOutput("Flywheel/Bottom Setpoint Reached", bottomController.atSetpoint());
        Logger.recordOutput("Flywheel/atSetpoint", this.atSetpoint());
        
        if(Robot.isSimulation()) {
            io.updateSimulation();
        }

        topController.setSetpoint(rpm);
        bottomController.setSetpoint(rpm);
        
        if(ControlConstants.kFlywheelPID) {
            double topOutput = topController.calculate(inputs.topVelocity);
            double bottomOutput = bottomController.calculate(inputs.bottomVelocity);
            // topOutput = 0;
            // bottomOutput = 0;

            if(topOutput < -3) topOutput = -3;
            if(bottomOutput < -3) bottomOutput = -3;

            Logger.recordOutput("Flywheel/TopOutput", topOutput + feedforward.calculate(rpm));
            Logger.recordOutput("Flywheel/BottomOutput", bottomOutput + feedforward.calculate(rpm));
            if(!driver.getRawButton(4)) {
                io.setTopVoltage(topOutput + feedforward.calculate(rpm));
                io.setBottomVoltage(bottomOutput + feedforward.calculate(rpm));
            } else {
                io.setTopVoltage(0);
                io.setBottomVoltage(0);
            }
        }
    }

    public void setSpeed(double speed) {
        setTopSpeed(speed);
        setBottomSpeed(speed);
    }

    public void setTopSpeed(double speed) {
        io.setTopSpeed(speed);
    }

    public void setBottomSpeed(double speed) {
        io.setBottomSpeed(speed);
    }

    public void setRPM(double rpm) {
        this.rpm = rpm;
        Logger.recordOutput("Flywheel/Setpoint", this.rpm);
    }

    public double getVelocity() {
        return io.getVelocity();
    }

    public boolean atSetpoint() {
        return (Math.abs(
            inputs.topVelocity - rpm)
                < 50.0) &&
            (Math.abs(
                inputs.bottomVelocity - rpm)
                < 50.0);
        // return (topController.atSetpoint() && bottomController.atSetpoint());
    }
}

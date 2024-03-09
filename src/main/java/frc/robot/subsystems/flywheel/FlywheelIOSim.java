// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class FlywheelIOSim implements FlywheelIO {
    
    FlywheelSim topFlywheel;
    FlywheelSim bottomFlywheel;

    double topVoltage = 0.0;
    double bottomVoltage = 0.0;

    // CANSparkMax topFlywheelMotor = new CANSparkMax()

    public FlywheelIOSim() {
        topFlywheel = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.002);
        bottomFlywheel = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.002);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.topVelocity = topFlywheel.getAngularVelocityRPM();
        inputs.bottomVelocity = bottomFlywheel.getAngularVelocityRPM();

        inputs.topVoltage = topVoltage;
        inputs.bottomVoltage = bottomVoltage;
    }

    @Override
    public void updateSimulation() {
        topFlywheel.update(0.02);
        bottomFlywheel.update(0.02);
    }

    @Override
    public void setTopSpeed(double speed) {
        double voltage = speed * RobotController.getBatteryVoltage();
        if(voltage < -RobotController.getBatteryVoltage()) voltage = -RobotController.getBatteryVoltage();
        if(RobotController.getBatteryVoltage() < voltage) voltage = RobotController.getBatteryVoltage();
        Logger.recordOutput("Flywheel/Sim Top Input Voltage", voltage);
        topVoltage = voltage;
        topFlywheel.setInputVoltage(voltage);
    }

    @Override
    public void setBottomSpeed(double speed) {
        double voltage = speed * RobotController.getBatteryVoltage();
        if(voltage < -RobotController.getBatteryVoltage()) voltage = -RobotController.getBatteryVoltage();
        if(RobotController.getBatteryVoltage() < voltage) voltage = RobotController.getBatteryVoltage();
        Logger.recordOutput("Flywheel/Sim Bottom Input Voltage", voltage);
        bottomVoltage = voltage;
        bottomFlywheel.setInputVoltage(voltage);
    }

    @Override
    public void setTopVoltage(double voltage) {
        if(RobotController.getBatteryVoltage() < voltage) voltage = RobotController.getBatteryVoltage();
        if(voltage < -RobotController.getBatteryVoltage()) voltage = -RobotController.getBatteryVoltage();
        topVoltage = voltage;
        topFlywheel.setInputVoltage(voltage);
    }

    @Override
    public void setBottomVoltage(double voltage) {
        if(RobotController.getBatteryVoltage() < voltage) voltage = RobotController.getBatteryVoltage();
        if(voltage < -RobotController.getBatteryVoltage()) voltage = -RobotController.getBatteryVoltage();
        bottomVoltage = voltage;
        bottomFlywheel.setInputVoltage(voltage);
    }

    @Override
    public double getVelocity() {
        return (topFlywheel.getAngularVelocityRPM() + bottomFlywheel.getAngularVelocityRPM()) / 2;
    }
}

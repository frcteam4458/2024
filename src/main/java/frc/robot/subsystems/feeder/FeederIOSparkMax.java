// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.HardwareConstants;

/** Add your docs here. */
public class FeederIOSparkMax implements FeederIO {
    CANSparkMax feeder;
    RelativeEncoder encoder;

    DigitalInput topSwitch;
    DigitalInput bottomSwitch;

    public FeederIOSparkMax() {
        feeder = new CANSparkMax(HardwareConstants.kFeeder, MotorType.kBrushless);
        encoder = feeder.getEncoder();

        topSwitch = new DigitalInput(HardwareConstants.kFeederTopSensor);
        bottomSwitch = new DigitalInput(HardwareConstants.kFeederBottomSensor);

        feeder.restoreFactoryDefaults();
        feeder.setSmartCurrentLimit(20);
        encoder.setPositionConversionFactor(0.05);
        feeder.setOpenLoopRampRate(0);
        feeder.setClosedLoopRampRate(0);

        feeder.setInverted(true);

        if(Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(feeder, DCMotor.getNeo550(1));
            SmartDashboard.putBoolean("Top Switch Sim", false);
        }
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.voltage = feeder.get() * RobotController.getBatteryVoltage();
        inputs.value = feeder.get();
        inputs.current = feeder.getOutputCurrent();
        inputs.rpm = encoder.getVelocity();
        inputs.position = encoder.getPosition();
        inputs.topSensor = !topSwitch.get();
        inputs.bottomSensor = !bottomSwitch.get();

        if(Robot.isSimulation()) {
            inputs.topSensor = SmartDashboard.getBoolean("Top Switch Sim", false);
        }
    }

    @Override
    public void setVoltage(double volts) {
        feeder.setVoltage(volts);
    }

    @Override
    public void set(double value) {
        feeder.set(value);
    }
}

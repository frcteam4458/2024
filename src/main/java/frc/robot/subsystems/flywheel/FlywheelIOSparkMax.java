// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.HardwareConstants;;

/** Add your docs here. */
public class FlywheelIOSparkMax implements FlywheelIO {
    CANSparkMax topMotor;
    CANSparkMax bottomMotor;

    RelativeEncoder topEncoder;
    RelativeEncoder bottomEncoder;

    public FlywheelIOSparkMax() {
        topMotor = new CANSparkMax(HardwareConstants.kFlywheelTop, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(HardwareConstants.kFlywheelBottom, MotorType.kBrushless);
        topEncoder = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();

        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();
        topMotor.setSmartCurrentLimit(40);
        bottomMotor.setSmartCurrentLimit(40);

        // topMotor.getPIDController().setref
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.topVoltage = topMotor.get() * RobotController.getBatteryVoltage();
        inputs.bottomVoltage = bottomMotor.get() * RobotController.getBatteryVoltage();

        inputs.topVelocity = topEncoder.getVelocity();
        inputs.bottomVelocity = bottomEncoder.getVelocity();
    }

    @Override
    public void setTopSpeed(double speed) {
        topMotor.set(speed);
    }

    @Override
    public void setBottomSpeed(double speed) {
        bottomMotor.set(speed);
    }

    @Override
    public void setTopVoltage(double voltage) {
        topMotor.setVoltage(voltage);
    }

    @Override
    public void setBottomVoltage(double voltage) {
        bottomMotor.setVoltage(voltage);
    }

    @Override
    public double getVelocity() {
        return (topEncoder.getVelocity() + bottomEncoder.getVelocity()) / 2;
    }
}

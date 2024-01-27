package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.HardwareConstants;

public class ArmIOSparkMax implements ArmIO {
    
    CANSparkMax armMotor;
    CANSparkMax armMotorFollower;
    RelativeEncoder armEncoder;

    public ArmIOSparkMax() {
        armMotor = new CANSparkMax(HardwareConstants.kArmMotor,
            MotorType.kBrushless);
        armMotorFollower = new CANSparkMax(HardwareConstants.kArmMotorFollower,
            MotorType.kBrushless);
        armMotorFollower.getEncoder();

        armMotor.restoreFactoryDefaults();
        armMotorFollower.restoreFactoryDefaults();

        armEncoder = armMotor.getEncoder();
        armEncoder.setPositionConversionFactor(
            HardwareConstants.kArmPositionConversionFactor);

        armMotorFollower.follow(armMotor);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.voltage = armMotor.get() * RobotController.getBatteryVoltage();
        inputs.angle = armEncoder.getPosition();
    }

    @Override
    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    @Override
    public double getAngle() {
        return armEncoder.getPosition();
    }

}

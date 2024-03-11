package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.HardwareConstants;

public class ArmIOSparkMax implements ArmIO {
    
    CANSparkMax armMotor;
    CANSparkMax armMotorFollower;
    RelativeEncoder armEncoder;

    DutyCycleEncoder absoluteEncoder;

    public ArmIOSparkMax() {
        armMotor = new CANSparkMax(HardwareConstants.kArmMotor,
            MotorType.kBrushless);
        armMotorFollower = new CANSparkMax(HardwareConstants.kArmMotorFollower,
            MotorType.kBrushless);
        armMotorFollower.getEncoder();

        armMotor.restoreFactoryDefaults();
        armMotorFollower.restoreFactoryDefaults();

        armMotor.setSmartCurrentLimit(40);
        armMotorFollower.setSmartCurrentLimit(40);

        armEncoder = armMotor.getEncoder();
        armEncoder.setPositionConversionFactor(
            HardwareConstants.kArmPositionConversionFactor);

        absoluteEncoder = new DutyCycleEncoder(HardwareConstants.kArmAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(HardwareConstants.kArmAbsoluteEncoderOffset + (1.0 / 360.0));
        absoluteEncoder.setDistancePerRotation(360);

        armMotor.setInverted(false);
        armMotorFollower.setInverted(true);
        // armMotorFollower.follow(armMotor);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.voltage = (armMotor.getAppliedOutput() * armMotor.getBusVoltage() + armMotorFollower.getAppliedOutput() * armMotorFollower.getBusVoltage()) / 2;
        inputs.angle = armEncoder.getPosition();
        inputs.absoluteAngle = getAngle();
    }

    @Override
    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
        armMotorFollower.setVoltage(volts);
    }

    @Override
    public double getAngle() {
        return absoluteEncoder.getDistance();
    }

    @Override
    public void setCoast(boolean coast) {
        armMotor.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
        armMotorFollower.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
    }

    public boolean getCoast(int motor) {
        if(motor == 0) return armMotor.getIdleMode() == IdleMode.kCoast;
        else return armMotorFollower.getIdleMode() == IdleMode.kCoast;
    }

}

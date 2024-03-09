package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.HardwareConstants;

public class ArmIOSim extends ArmIOSparkMax {

    SingleJointedArmSim armSim;

    EncoderSim encoderSim;

    DutyCycleEncoderSim dutyCycleEncoderSim;

    double input = 0.0;

    public ArmIOSim() {
        super();

        dutyCycleEncoderSim = new DutyCycleEncoderSim(super.absoluteEncoder);

        armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            200.0,
            SingleJointedArmSim.estimateMOI(HardwareConstants.kArmLength, HardwareConstants.kArmMass),
            HardwareConstants.kArmLength,
            0,
            (3.0 * Math.PI) / 4.0,
            true,
            0.0
        );
        System.out.println("Arm MOI: " + SingleJointedArmSim.estimateMOI(HardwareConstants.kArmLength, HardwareConstants.kArmMass));
    }

    @Override
    public void updateSim() {
        armSim.setInput(-input);

        armSim.update(0.02);

        armEncoder.setPosition(Math.toDegrees(armSim.getAngleRads()));
        // dutyCycleEncoderSim.setAbsolutePosition(armEncoder.getPosition());
        dutyCycleEncoderSim.setDistance(armEncoder.getPosition());
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        super.updateInputs(inputs);
        inputs.absoluteAngle = armEncoder.getPosition();
    }

    // @Override
    // public double getAngle() {
    //     return dutyCycleEncoderSim.getDistance();
    // }

    @Override
    public void setVoltage(double volts) {
        input = volts;
    }
}

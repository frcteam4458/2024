package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.HardwareConstants;

public class ArmIOSim extends ArmIOSparkMax {

    SingleJointedArmSim armSim;

    EncoderSim encoderSim;

    double input = 0.0;

    public ArmIOSim() {
        super();

        armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            200.0,
            SingleJointedArmSim.estimateMOI(HardwareConstants.kArmLength, HardwareConstants.kArmMass),
            HardwareConstants.kArmLength,
            -1,
            4.0,
            true,
            0.0
        );
    }

    @Override
    public void updateSim() {
        armSim.setInput(input);

        armSim.update(0.02);

        armEncoder.setPosition(armSim.getAngleRads());
    }

    @Override
    public void setVoltage(double volts) {
        input = volts;
    }
}

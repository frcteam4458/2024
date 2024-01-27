package frc.robot.subsystems.arm;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
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
            SingleJointedArmSim.estimateMOI(0.5, HardwareConstants.kArmMass),
            0.5,
            -3.14,
            3.14,
            true,
            1.0
        );

        // REVPhysicsSim.getInstance().addSparkMax(armMotor, DCMotor.getNEO(1));
        // REVPhysicsSim.getInstance().addSparkMax(armMotorFollower, DCMotor.getNEO(1));
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

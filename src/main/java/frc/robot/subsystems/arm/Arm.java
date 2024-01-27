package frc.robot.subsystems.arm;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.PIDControlConstants;

public class Arm extends SubsystemBase {

    PIDController controller;
    Mechanism2d mechanism;
    MechanismLigament2d armLigament;

    ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    ArmIO io;

    double setpoint = 0;

    public Arm(ArmIO io) {
        this.io = io;

        controller = new PIDController(
            PIDControlConstants.kArmP,
            PIDControlConstants.kArmI,
            PIDControlConstants.kArmD);

        mechanism = new Mechanism2d(0, 0);
        MechanismRoot2d root = mechanism.getRoot("shooter", -HardwareConstants.kYOriginToArm, HardwareConstants.kZOriginToArm);
        armLigament = root.append(new MechanismLigament2d("arm", HardwareConstants.kArmLength, 90));
        armLigament.setLineWeight(5);
        armLigament.setColor(new Color8Bit(128, 0, 128));

        new Trigger(DriverStation::isDisabled).whileTrue(new InstantCommand(new Runnable() {
            @Override
            public void run() {
                setSetpoint(io.getAngle());
            }
        }));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        armLigament.setAngle(io.getAngle() * (180.0 / Math.PI));
        Logger.recordOutput("Arm/Mechanism", mechanism);
        Logger.recordOutput("Arm/Setpoint", setpoint);
        Logger.recordOutput("Arm/Pose3d", new Pose3d(-HardwareConstants.kYOriginToArm, 0, HardwareConstants.kZOriginToArm, new Rotation3d((Math.PI / 2.0) - getAngleRad(), 0.0, (Math.PI / 2.0))));

        if(PIDControlConstants.kArmPid) {
            setVoltage(controller.calculate(io.getAngle()));
        }
    }

    @Override
    public void simulationPeriodic() {
        io.updateSim();
    }

    public void setSetpoint(double angle) {
        setpoint = angle;
        controller.setSetpoint(setpoint);
    }

    public void setMotor(double value) {
        setVoltage(value * RobotController.getBatteryVoltage());
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void adjustSetpoint(double adjustment) {
        setSetpoint(setpoint += (adjustment * (HardwareConstants.kArmRadPerSec / 50)));
    }

    public void setInput(double input) {
        if(PIDControlConstants.kArmPid) {
            adjustSetpoint(input);
        } else {
            setMotor(input);
        }
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(io.getAngle());
    }

    public double getAngleRad() {
        return io.getAngle();
    }

}

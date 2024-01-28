package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.Constants.ControlConstants;

public class Arm extends SubsystemBase {

    ProfiledPIDController profiledPIDController;
    PIDController pidController;

    Mechanism2d mechanism;
    MechanismLigament2d armLigament;
    MechanismLigament2d setpointLigament;

    ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    ArmIO io;

    double setpoint = 0;

    boolean manual = false;

    public Arm(ArmIO io) {
        this.io = io;

        pidController = new PIDController(
            ControlConstants.kArmP,
            ControlConstants.kArmI,
            ControlConstants.kArmD);

        profiledPIDController = new ProfiledPIDController(
            ControlConstants.kArmP,
            ControlConstants.kArmI,
            ControlConstants.kArmD, 
            new TrapezoidProfile.Constraints(
                ControlConstants.kArmRadPerSec, ControlConstants.kArmAccel));

        mechanism = new Mechanism2d(0, 0);
        MechanismRoot2d root = mechanism.getRoot("shooter", -HardwareConstants.kYOriginToArm, HardwareConstants.kZOriginToArm);
        armLigament = root.append(new MechanismLigament2d("arm", HardwareConstants.kArmLength, 90));
        armLigament.setLineWeight(5);
        armLigament.setColor(new Color8Bit(128, 0, 128));
        setpointLigament = root.append(new MechanismLigament2d("armSetpoint", HardwareConstants.kArmLength, 90));
        setpointLigament.setLineWeight(2);
        setpointLigament.setColor(new Color8Bit(0, 255, 0));

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
        setpointLigament.setAngle(setpoint * (180.0 / Math.PI));
        Logger.recordOutput("Arm/Mechanism", mechanism);
        Logger.recordOutput("Arm/Setpoint", setpoint);
        Logger.recordOutput("Arm/Pose3d", new Pose3d(-HardwareConstants.kYOriginToArm, 0, HardwareConstants.kZOriginToArm, new Rotation3d((Math.PI / 2.0) - getAngleRad(), 0.0, (Math.PI / 2.0))));
        Logger.recordOutput("Arm/SetpointPose3d", new Pose3d(-HardwareConstants.kYOriginToArm, 0, HardwareConstants.kZOriginToArm, new Rotation3d((Math.PI / 2.0) - setpoint, 0.0, (Math.PI / 2.0))));


        // better way to do the switching might be ProfiledPIDController::setConstraints?
        if(ControlConstants.kArmPid) {
            double output = 0.0;
            output = profiledPIDController.calculate(io.getAngle());
            if(manual)
                output = pidController.calculate(io.getAngle());
            setVoltage(output);
        }
    }

    @Override
    public void simulationPeriodic() {
        io.updateSim();
    }

    public void setSetpoint(double angle) {
        setpoint = angle;
        if(setpoint < HardwareConstants.kArmRotPhysicalMin) {
            setpoint = HardwareConstants.kArmRotPhysicalMin;
        }
        if(HardwareConstants.kArmRotPhysicalMax < setpoint) {
            setpoint = HardwareConstants.kArmRotPhysicalMax;
        }
        pidController.setSetpoint(setpoint);
        profiledPIDController.setGoal(setpoint);
    }

    public void setMotor(double value) {
        setVoltage(value * RobotController.getBatteryVoltage());
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void adjustSetpoint(double adjustment) {
        setSetpoint(setpoint += (adjustment * (ControlConstants.kArmRadPerSec / 50)));
    }

    public void setInput(double input) {
        if(ControlConstants.kArmPid) {
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

    public void setScoringSetpoint(double distance) {
        setSetpoint((-0.0386552 * distance * distance) + (0.350685 * distance) + 0.0935409);
    }

    public void setManualControl(boolean manualControl) {
        manual = manualControl;
    }

}

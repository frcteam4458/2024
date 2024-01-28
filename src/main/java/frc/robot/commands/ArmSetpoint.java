package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.arm.Arm;

public class ArmSetpoint extends InstantCommand {
  Arm arm;
  double setpoint;
  public ArmSetpoint(Arm arm, double setpoint, String name) {
    this.arm = arm;
    this.setpoint = setpoint;
    this.setName(name);
  }

  @Override
  public void initialize() {
    arm.setSetpoint(setpoint);
  }

  public static void registerNamedSetpoints(Arm arm) {
    NamedCommands.registerCommand("ArmAmp", new ArmSetpoint(arm, PositionConstants.kAmpArmAngle, "ArmAmp"));
    NamedCommands.registerCommand("ArmSource", new ArmSetpoint(arm, PositionConstants.kSourceArmAngle, "ArmSource"));
  }
}

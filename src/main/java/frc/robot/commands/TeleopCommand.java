// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.Robot;

public class TeleopCommand extends Command {

  DriveSubsystem driveSubsystem;
  XboxController controller;
  SlewRateLimiter filterX;
  SlewRateLimiter filterY;

  boolean fieldOrinted = true;
  CommandXboxController commandXboxController;

  public TeleopCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    controller = new XboxController(0);
    commandXboxController = new CommandXboxController(0);
    addRequirements(driveSubsystem);
    filterX = new SlewRateLimiter(OperatorConstants.kDriveRateLimit);
    filterY = new SlewRateLimiter(OperatorConstants.kDriveRateLimit);
  }

  @Override
  public void initialize() {
    driveSubsystem.arcadeDrive(0, 0, 0);

    commandXboxController.pov(270).onTrue(new InstantCommand(new Runnable() {

      @Override
      public void run() {
        fieldOrinted = !fieldOrinted;
      }
      
    }));
  }

  @Override
  public void execute() {
    // driveSubsystem.arcadeDrive(getX(), getY(), getOmega());
    if(fieldOrinted)
      driveSubsystem.arcadeDriveFieldOriented(getX(), getY(), getOmega());
    else
      driveSubsystem.arcadeDrive(getX(), getY(), getOmega());
      
    Logger.recordOutput("TeleopCommand/Field Orinted", fieldOrinted);
  }

  public double getX() {
    double filteredX = 0.0; // Forward/backward
    if (!Robot.isReal()) filteredX = filterX.calculate(-controller.getRawAxis(1)); // Simulated axis
    else filteredX = filterX.calculate(-controller.getLeftY()); // Real axis
    if (Math.abs(filteredX) < 0.1) filteredX = 0; // Deadzone
    return filteredX;
  }

  public double getY() {
    double filteredY = 0.0;
    if (!Robot.isReal()) filteredY = filterY.calculate(-controller.getRawAxis(0));
    else filteredY = filterY.calculate(-controller.getLeftX());
    if (Math.abs(filteredY) < 0.1) filteredY = 0;
    return filteredY;
  }

  public double getOmega() {
    double omega = 0.0; // Turning
    if (!Robot.isReal()) omega = -controller.getRawAxis(2);
    else omega = -controller.getRightX();
    if (Math.abs(omega) < 0.1) omega = 0;
    return omega;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

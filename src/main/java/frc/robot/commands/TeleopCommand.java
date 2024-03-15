// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.Robot;

public class TeleopCommand extends Command {

  DriveSubsystem driveSubsystem;
  XboxController controller;
  GenericHID genericController;
  SlewRateLimiter filterX;
  SlewRateLimiter filterY;

  Debouncer joystickDebouncer;

  // boolean fieldOrinted = true;
  CommandXboxController commandXboxController;


  public TeleopCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    controller = new XboxController(0);
    commandXboxController = new CommandXboxController(0);
    genericController = new GenericHID(0);
    addRequirements(driveSubsystem);
    filterX = new SlewRateLimiter(OperatorConstants.kDriveRateLimit);
    filterY = new SlewRateLimiter(OperatorConstants.kDriveRateLimit);

    joystickDebouncer = new Debouncer(0.5);
  }

  @Override
  public void initialize() {
    driveSubsystem.arcadeDrive(0, 0, 0);
  }

  @Override
  public void execute() {
    double x = getX();
    double y = getY();
    double omega = getOmega();
    boolean lock = false;

    // if((Math.abs(x) < 0.01) && (Math.abs(y) < 0.01) && (Math.abs(omega) < 0.01)) lock = joystickDebouncer.calculate(true);
    // else lock = joystickDebouncer.calculate(false);

    if(lock) {
      driveSubsystem.lock();
      return;
    }

    if(genericController.getRawButton(5))
      driveSubsystem.arcadeDriveFieldOriented(getX(), getY(), getOmega());
    else
     driveSubsystem.arcadeDrive(getX(), getY(), getOmega());
  }

  public double getX() {
    double filteredX = 0.0; // Forward/backward
    if (!Robot.isReal()) filteredX = filterX.calculate(-genericController.getRawAxis(1)); // Simulated axis
    else filteredX = -controller.getLeftY(); // Real axis
    if (Math.abs(filteredX) < 0.01) filteredX = 0; // Deadzone
    Logger.recordOutput("TeleopCommand/X", filteredX);
    return filteredX;
  }

  public double getY() {
    double filteredY = 0.0;
    if (!Robot.isReal()) filteredY = filterY.calculate(-genericController.getRawAxis(0));
    else filteredY = -controller.getLeftX();
    if (Math.abs(filteredY) < 0.01) filteredY = 0;
    Logger.recordOutput("TeleopCommand/Y", filteredY);
    return filteredY;
  }

  public double getOmega() {
    double omega = 0.0; // Turning
    if (!Robot.isReal()) omega = -genericController.getRawAxis(2);
    if (Robot.isReal()) omega = -genericController.getRawAxis(2);
    // else omega = -controller.getRightX();
    if (Math.abs(omega) < 0.01) omega = 0;
    Logger.recordOutput("TeleopCommand/Omega", omega);
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

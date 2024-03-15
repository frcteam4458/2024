// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.VirtualSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.REVPhysicsSim;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    if (Robot.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
      // Logger.addDataReceiver(new NT4Publisher());
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Logger.recordOutput("Battery Voltage", RobotController.getBatteryVoltage());
    VirtualSubsystem.periodicAll();

    Translation2d speaker = PositionConstants.kSpeakerPosition.plus(new Translation2d(0.5, 0));
            Translation2d robot = DriveSubsystem.robotPose.getTranslation();

            double dist = robot.getDistance(speaker);

    Logger.recordOutput("dist", dist);
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}

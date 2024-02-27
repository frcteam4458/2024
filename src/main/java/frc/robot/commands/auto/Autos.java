// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.RobotContainer;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Add your docs here. */
public class Autos {

  private static DriveSubsystem driveSubsystem;

  public static void constructAutoBuilder(DriveSubsystem _driveSubsystem) {
    driveSubsystem = _driveSubsystem;

    AutoBuilder.configureHolonomic(
        driveSubsystem::getPose,
        driveSubsystem::setPose,
        driveSubsystem::getChassisSpeeds,
        driveSubsystem::driveChassisSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(HardwareConstants.kP, 0, 0),
            new PIDConstants(HardwareConstants.kAngularP, 0, 0),
            HardwareConstants.kMaxSpeed,
            HardwareConstants.kDriveBaseRadius,
            new ReplanningConfig(true, true)),
        RobotContainer::isRed,
        driveSubsystem);
  }

  public static boolean getFlip() {
    return RobotContainer.isRed();
  }
}

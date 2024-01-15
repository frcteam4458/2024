// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

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
            new PIDConstants(OperatorConstants.kP, 0, 0),
            new PIDConstants(OperatorConstants.kAngularP, 0, 0),
            OperatorConstants.kMaxSpeed,
            OperatorConstants.kDriveBaseRadius,
            new ReplanningConfig()),
        RobotContainer::isRed,
        driveSubsystem);
  }

  public static Command B1R3_2Cube() {
    return new B1R3_2Cube(driveSubsystem);
  }

  public static boolean getFlip() {
    return false;
  }
}

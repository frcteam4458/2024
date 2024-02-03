// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.ArmSetpoint;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.PIDAlign;
import frc.robot.commands.auto.SpeakerAlign;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemIOSparkMax;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final CommandXboxController driverController
    = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick operatorJoystick
    = new CommandJoystick(OperatorConstants.kOperartorControllerPort);
  private DriveSubsystem driveSubsystem;
  private Arm arm;

  private VisionSubsystem visionSubsystem;

  private TeleopCommand teleopCommand;

  private SendableChooser<Integer> autoChooser = new SendableChooser<Integer>();

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem(new DriveSubsystemIOSparkMax());
    if(Robot.isReal()) arm = new Arm(new ArmIOSparkMax());
    else arm = new Arm(new ArmIOSim());

    ArmSetpoint.registerNamedSetpoints(arm);

    // visionSubsystem = new VisionSubsystem();

    Autos.constructAutoBuilder(driveSubsystem);

    teleopCommand = new TeleopCommand(driveSubsystem);

    autoChooser.setDefaultOption("Nothing", 0);
    autoChooser.addOption("Amp Station Auto", 1);
    autoChooser.addOption("Quasistatic Forward", 2);
    autoChooser.addOption("Quasistatic Backward", 3);
    autoChooser.addOption("Dynamic Forward", 4);
    autoChooser.addOption("Dynamic Backward", 5);

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

    driveSubsystem.setPose(0, 0, 0);
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(teleopCommand);

    buttonTriggerPIDAlign(driverController.a(), PositionConstants.kAmpPose, RobotContainer::isRed,
      ControlConstants.kP, ControlConstants.kI, ControlConstants.kD,
      ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
    );

    driverController.a().onTrue(NamedCommands.getCommand("ArmAmp"));
    
    buttonTriggerPIDAlign(driverController.x(), PositionConstants.kSource1Pose, RobotContainer::isBlue,
      ControlConstants.kP, ControlConstants.kI, ControlConstants.kD,
      ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
    );
    driverController.x().onTrue(NamedCommands.getCommand("ArmSource"));

    buttonTriggerPIDAlign(driverController.y(), PositionConstants.kSource2Pose, RobotContainer::isBlue,
      ControlConstants.kP, ControlConstants.kI, ControlConstants.kD,
      ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
    );
    driverController.y().onTrue(NamedCommands.getCommand("ArmSource"));

    buttonTriggerPIDAlign(driverController.b(), PositionConstants.kSource3Pose, RobotContainer::isBlue,
      ControlConstants.kP, ControlConstants.kI, ControlConstants.kD,
      ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
    );
    driverController.b().onTrue(NamedCommands.getCommand("ArmSource"));

    driverController.rightBumper().whileTrue(
      new SpeakerAlign(
        driveSubsystem, visionSubsystem, RobotContainer::isRed,
        ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
      )
    );

    NamedCommands.registerCommand("AlignSpeaker", new SpeakerAlign(
        driveSubsystem, visionSubsystem, RobotContainer::isRed,
        ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
      ));
    
    driverController.rightBumper().whileTrue(Commands.run(new Runnable() {
      @Override
      public void run() {
        Pose2d pose = driveSubsystem.getPose();
        Translation2d speaker = PositionConstants.kSpeakerPosition;
        arm.setScoringSetpoint(
          Math.sqrt(
            Math.pow(
              pose.getX() - speaker.getX(), 2)) +
            Math.pow(
              pose.getY() - speaker.getY(), 2));
      }
    }, arm));

    driverController.leftBumper().onTrue(NamedCommands.getCommand("ArmFloor"));

    // Reset Gyro
    driverController.pov(0).onTrue(new InstantCommand(new Runnable() {
      @Override
      public void run() {
        driveSubsystem.setPose(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY(), 0);
      }
    }) );

    new Trigger(() -> {
      return Math.abs(operatorJoystick.getRawAxis(0)) > 0.1;
    }).onTrue(Commands.runOnce(new Runnable() {
      @Override
      public void run() {
        arm.setManualControl(true);
      }
    }, arm)).whileTrue(Commands.run(new Runnable() {
      @Override
      public void run() {
        arm.setInput(operatorJoystick.getRawAxis(0));
      }
    }, arm)).onFalse(Commands.runOnce(new Runnable() {
      @Override
      public void run() {
        arm.setInput(0);
        arm.setManualControl(false);
      }
    }));

  }

  public Command getAutonomousCommand() {
    if(autoChooser.getSelected() == 0) {
      return Commands.none();
    } else if(autoChooser.getSelected() == 1) {
      return Autos.B1R3_2Cube();
    } else if(autoChooser.getSelected() == 2) {
      return driveSubsystem.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    } else if(autoChooser.getSelected() == 3) {
      return driveSubsystem.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    } else if(autoChooser.getSelected() == 4) {
      return driveSubsystem.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    } else if(autoChooser.getSelected() == 5) {
      return driveSubsystem.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    return Commands.none();
  }

  /**
   * Defaults to blue
   * @return Alliance
   */
  public static Alliance getAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if(alliance.isPresent()) {
      return alliance.get();
    }
    else return Alliance.Blue;
  }

  public static boolean isRed() {
    return getAlliance() == Alliance.Red;
  }

  public static boolean isBlue() {
    return getAlliance() == Alliance.Blue;
  }

  public static boolean isInvalid() {
    return DriverStation.getAlliance().isEmpty();
  }

  public void buttonTriggerPIDAlign(Trigger trigger, Pose2d target, BooleanSupplier flip, double p, double i, double d, double ap, double ai, double ad) {
    trigger.whileTrue(new PIDAlign(driveSubsystem, target, flip, p, i, d, ap, ai, ad));
  }
}

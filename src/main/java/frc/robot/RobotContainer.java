// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.ArmSetpoint;
import frc.robot.commands.FlywheelCommand;
import frc.robot.commands.Intake;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.PIDAlign;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemIOSparkMax;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSparkMax;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final CommandXboxController driverController
    = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick operatorJoystick
    = new CommandJoystick(OperatorConstants.kOperartorControllerPort);

  private DriveSubsystem driveSubsystem;
  private Arm arm;
  private Flywheel flywheel;
  private Feeder feeder;
  private Climb climb;

  private VisionSubsystem visionSubsystem;

  private TeleopCommand teleopCommand;
  private SequentialCommandGroup shootCommand;
  private Intake intakeCommand;
  private SequentialCommandGroup intakeCommandGroup;

  private SendableChooser<Integer> autoChooser = new SendableChooser<Integer>();

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem(new DriveSubsystemIOSparkMax());
    if(Robot.isReal()) {
      arm = new Arm(new ArmIOSparkMax());
      flywheel = new Flywheel(new FlywheelIOSparkMax());
    }
    else {
      arm = new Arm(new ArmIOSim());
      flywheel = new Flywheel(new FlywheelIOSim());
    }

    feeder = new Feeder(new FeederIOSparkMax());
    // climb = new Climb();

    intakeCommand = new Intake(feeder);

    ArmSetpoint.registerNamedSetpoints(arm);

    visionSubsystem = new VisionSubsystem();

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

    SmartDashboard.putNumber("Flywheel RPM", 0);
    SmartDashboard.putNumber("Feeder", 0);
  }



  private void configureBindings() {
    driveSubsystem.setDefaultCommand(teleopCommand);

    configureDriveSetpoints();
    configureArmJoystick();

    // flywheel.setDefaultCommand(Commands.run(() -> {
    //   flywheel.setRPM(SmartDashboard.getNumber("Flywheel RPM", 0));
    // }, flywheel));

    // operatorJoystick.button(7).onTrue(new InstantCommand(() -> {
    //   driveSubsystem.setPose(new Pose2d());
    // }));

    // operatorJoystick.button(11).onTrue(new InstantCommand(() -> {
    //   climb.set(true);
    // }, climb));

    // operatorJoystick.button(10).onTrue(new InstantCommand(() -> {
    //   climb.set(false);
    // }, climb));

    // new Trigger(DriverStation::isEnabled).onTrue(new InstantCommand(() -> {
    //   flywheel.setRPM(1000);
    // }, flywheel));

    SmartDashboard.putBoolean("Toggle Flywheel/Feeder", false);

      intakeCommandGroup = new SequentialCommandGroup(
        new InstantCommand(() -> {
          arm.setSetpoint(0);
        }, arm),
        new Intake(feeder),
        new WaitCommand(0.25),
        new InstantCommand(() -> {
          feeder.setSetpoint(feeder.getPosition() + 0.75);
        }, feeder)
      );

    NamedCommands.registerCommand("Intake", intakeCommandGroup);

    shootCommand = new SequentialCommandGroup(
      new FlywheelCommand(flywheel, PositionConstants.kShootVelocity),
      new InstantCommand(() -> {
          feeder.set(ControlConstants.kFeederMagnitude);
        }, feeder
      ),
      new WaitCommand(0.1),
      new InstantCommand(() -> {
          feeder.set(0);
        }, feeder
      ),
      new ParallelCommandGroup(
        new FlywheelCommand(flywheel, 1000),
        intakeCommandGroup
      )
    );

    NamedCommands.registerCommand("Shoot", shootCommand);

    operatorJoystick.button(1).onTrue(NamedCommands.getCommand("Shoot"));
    operatorJoystick.button(2).onTrue(NamedCommands.getCommand("Intake"));



    registerNamedCommands();
  }

  public Command getAutonomousCommand() {
    if(autoChooser.getSelected() == 0) {
      return Commands.none();
    } else if(autoChooser.getSelected() == 1) {
      return Autos.B1R3_2Cube();
    }
    
    else if(autoChooser.getSelected() == 2) {
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


  // Helper method for binding odometry setpoints
  public void buttonTriggerPIDAlign(Trigger trigger, Pose2d target, BooleanSupplier flip, double p, double i, double d, double ap, double ai, double ad) {
    trigger.whileTrue(new PIDAlign(driveSubsystem, target, flip, p, i, d, ap, ai, ad));
  }

  public void configureDriveSetpoints() {
    // buttonTriggerPIDAlign(driverController.a(), PositionConstants.kAmpPose, RobotContainer::isRed,
    //   ControlConstants.kP, ControlConstants.kI, ControlConstants.kD,
    //   ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
    // );
    // driverController.a().onTrue(NamedCommands.getCommand("ArmAmp"));
    
    // buttonTriggerPIDAlign(driverController.x(), PositionConstants.kSource1Pose, RobotContainer::isBlue,
    //   ControlConstants.kP, ControlConstants.kI, ControlConstants.kD,
    //   ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
    // );
    // driverController.x().onTrue(NamedCommands.getCommand("ArmSource"));

    // buttonTriggerPIDAlign(driverController.y(), PositionConstants.kSource2Pose, RobotContainer::isBlue,
    //   ControlConstants.kP, ControlConstants.kI, ControlConstants.kD,
    //   ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
    // );
    // driverController.y().onTrue(NamedCommands.getCommand("ArmSource"));

    // buttonTriggerPIDAlign(driverController.b(), PositionConstants.kSource3Pose, RobotContainer::isBlue,
    //   ControlConstants.kP, ControlConstants.kI, ControlConstants.kD,
    //   ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
    // );
    // driverController.b().onTrue(NamedCommands.getCommand("ArmSource"));
    // driverController.y().onTrue(new InstantCommand(() -> {
    //   arm.setSetpoint(90);
    // }, arm));

    // driverController.rightBumper().onTrue(new InstantCommand(() -> {
    //   arm.setSetpoint(PositionConstants.kSpeakerAngle);
    // }, arm));

    // driverController.rightBumper().whileTrue(
    //   new PIDAngle(
    //     driveSubsystem, visionSubsystem, RobotContainer::isRed,
    //     ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD,
    //     PositionConstants.kSpeakerPosition
    //   )
    // );
    
    // driverController.rightBumper().whileTrue(Commands.run(new Runnable() {
    //   @Override
    //   public void run() {
    //     Pose2d pose = driveSubsystem.getPose();
    //     Translation2d speaker = PositionConstants.kSpeakerPosition;
    //     arm.setScoringSetpoint(
    //       Math.sqrt(
    //         Math.pow(
    //           pose.getX() - speaker.getX(), 2)) +
    //         Math.pow(
    //           pose.getY() - speaker.getY(), 2));
    //   }
    // }, arm));

    // driverController.leftBumper().onTrue(NamedCommands.getCommand("ArmFloor"));

  }

  public void configureArmJoystick() {
    // Arm Control (probably make a command for this)
    new Trigger(() -> {
      return Math.abs(operatorJoystick.getRawAxis(0)) > 0.2;
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
    }, arm)).whileFalse(Commands.runOnce(new Runnable() {
      @Override
      public void run() {
        arm.setInput(0);
        arm.setManualControl(false);
      }
    }));
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("IntakeForward", new InstantCommand(() -> {
      feeder.set(ControlConstants.kFeederMagnitude);
    }));
    NamedCommands.registerCommand("IntakeReverse", new InstantCommand(() -> {
      feeder.set(-ControlConstants.kFeederMagnitude);
    }));
    NamedCommands.registerCommand("IntakeStop", new InstantCommand(() -> {
      feeder.set(0);
    }));
  }
}
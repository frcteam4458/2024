// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.ArmSetpoint;
import frc.robot.commands.FlywheelCommand;
import frc.robot.commands.HeadingCommand;
import frc.robot.commands.Intake;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.auto.AlignCommand;
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
  private final GenericHID genericController = new GenericHID(0);
  private final CommandGenericHID commandGeneric = new CommandGenericHID(0);

  private DriveSubsystem driveSubsystem;
  private Arm arm;
  private Flywheel flywheel;
  private Feeder feeder;
  private Climb climb;

  private VisionSubsystem visionSubsystem;

  private TeleopCommand teleopCommand;
  private Command shootCommand;
  private Command shootIntakeCommand;
  private Intake intakeCommand;
  private Command intakeCommandGroup;

  private SendableChooser<String> autoChooser = new SendableChooser<String>();

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem(new DriveSubsystemIOSparkMax());
    if(Robot.isReal()) {
      arm = new Arm(new ArmIOSparkMax());
      flywheel = new Flywheel(new FlywheelIOSparkMax());
    } else {
      arm = new Arm(new ArmIOSim());
      flywheel = new Flywheel(new FlywheelIOSim());
    }

    feeder = new Feeder(new FeederIOSparkMax());
    // climb = new Climb();

    intakeCommand = new Intake(feeder);

    ArmSetpoint.registerNamedSetpoints(arm);

    visionSubsystem = new VisionSubsystem();


    teleopCommand = new TeleopCommand(driveSubsystem);

    String[] autos = new String[]{
      "Nothing",
      "OTF",
      "2 Note Top",

      "Quasistatic Forward",
      "Quasistatic Backward",
      "Dynamic Forward",
      "Dynamic Backward",
    };

    for(String auto : autos) {
      autoChooser.addOption(auto, auto);
    }

    SmartDashboard.putData("Autonomous Chooser", autoChooser);

    configureBindings();
    Autos.constructAutoBuilder(driveSubsystem);

    driveSubsystem.setPose(0, 0, 0);

    SmartDashboard.putNumber("Flywheel RPM", 0);
  }



  private void configureBindings() {

    SmartDashboard.putData("Nuclear Button", Commands.runOnce(() -> {
      CommandScheduler.getInstance().cancelAll();
    }));

    driveSubsystem.setDefaultCommand(teleopCommand);

    configureDriveSetpoints();
    configureArmJoystick();

    // operatorJoystick.button(7).onTrue(new InstantCommand(() -> {
    //   driveSubsystem.setPose(new Pose2d());
    // }));

    // operatorJoystick.button(11).onTrue(new InstantCommand(() -> {
    //   climb.set(true);
    // }, climb));

    // operatorJoystick.button(10).onTrue(new InstantCommand(() -> {
    //   climb.set(false);
    // }, climb));

    intakeCommandGroup = Commands.sequence(
      new InstantCommand(() -> {
        arm.setSetpoint(0);
      }, arm)).withInterruptBehavior(InterruptionBehavior.kCancelSelf).asProxy().andThen(Commands.sequence(
      new Intake(feeder),
      new WaitCommand(0.25),
      new InstantCommand(() -> {
        feeder.setSetpoint(feeder.getPosition() + 0.75);
      }, feeder)
    )).withInterruptBehavior(InterruptionBehavior.kCancelSelf).asProxy().andThen(new InstantCommand(() -> {
      arm.setSetpoint(10);
    }));

    NamedCommands.registerCommand("Intake", intakeCommandGroup);
    NamedCommands.registerCommand("AutoIntake", Commands.runOnce(() -> {
      intakeCommandGroup.asProxy().schedule();
    }));

    shootCommand = Commands.parallel(
        new FlywheelCommand(flywheel, PositionConstants.kShootVelocity), // Put flywheel up to speed

        Commands.sequence(Commands.runOnce(() -> {
          arm.setSetpoint(20); 
          }, arm),
          Commands.run(() -> {

          }).until(() -> {
            return (Math.abs(arm.getSetpoint() - arm.getAngle()) < 2.0);
          })
        )
      )
      .andThen(Commands.sequence(
        new InstantCommand(() -> {
            feeder.set(ControlConstants.kFeederMagnitude);
          }, feeder
        ),
        new WaitCommand(0.2),
        new InstantCommand(() -> {
          feeder.set(0);
          flywheel.setRPM(1000);
        }, feeder, flywheel))
    );

    shootIntakeCommand = shootCommand.asProxy().andThen(Commands.runOnce(() -> {
      intakeCommandGroup.schedule();
    }));

    NamedCommands.registerCommand("Shoot", shootCommand);


    NamedCommands.registerCommand("AutoShoot", Commands.parallel(
        new FlywheelCommand(flywheel, PositionConstants.kShootVelocity), // Put flywheel up to speed

        Commands.sequence(Commands.runOnce(() -> {
          arm.setSetpoint(20); 
          }),
          Commands.run(() -> {

          }).until(() -> {
            return (Math.abs(arm.getSetpoint() - arm.getAngle()) < 2.0);
          })
        ).andThen(Commands.sequence(
        new InstantCommand(() -> {
            feeder.set(ControlConstants.kFeederMagnitude);
          }
        ),
        new WaitCommand(0.2),
        new InstantCommand(() -> {
          feeder.set(0);
          flywheel.setRPM(1000);
        }))
    )
    ));





    NamedCommands.registerCommand("ShootIntake", shootIntakeCommand);

    
    operatorJoystick.button(1).onTrue(NamedCommands.getCommand("ShootIntake"));
    commandGeneric.button(12).onTrue(NamedCommands.getCommand("ShootIntake"));
    operatorJoystick.button(2).onTrue(NamedCommands.getCommand("Intake"));

    commandGeneric.button(13).onTrue(new HeadingCommand(driveSubsystem, 0));
    commandGeneric.button(14).onTrue(new HeadingCommand(driveSubsystem, 90));
    commandGeneric.button(15).onTrue(new HeadingCommand(driveSubsystem, 180));
    commandGeneric.button(16).onTrue(new HeadingCommand(driveSubsystem, 270));

    NamedCommands.registerCommand("SpeakerAlign", new AlignCommand(driveSubsystem, visionSubsystem, RobotContainer::isRed, ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD));

    registerNamedCommands();
  }

  public Command getAutonomousCommand() {

    driveSubsystem.driveChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    if(autoChooser.getSelected().equalsIgnoreCase("Quasistatic Forward")) {
      return driveSubsystem.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    } else if(autoChooser.getSelected().equalsIgnoreCase("Quasistatic Backward")) {
      return driveSubsystem.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    } else if(autoChooser.getSelected().equalsIgnoreCase("Dynamic Forward")) {
      return driveSubsystem.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    } else if(autoChooser.getSelected().equalsIgnoreCase("Dynamic Backward")) {
      return driveSubsystem.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    } else if(autoChooser.getSelected().equalsIgnoreCase("Nothing")) {
      return Commands.none();
    } else {
      return new PathPlannerAuto(autoChooser.getSelected());
      // return AutoBuilder.buildAuto(autoChooser.getSelected()).andThen(Commands.runOnce(() -> { driveSubsystem.arcadeDrive(0, 0, 0); }));
    }
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
    buttonTriggerPIDAlign(driverController.a(), PositionConstants.kAmpPose, RobotContainer::isRed,
      ControlConstants.kP, ControlConstants.kI, ControlConstants.kD,
      ControlConstants.kAP, ControlConstants.kAI, ControlConstants.kAD
    );
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
    // new Trigger(() -> {
    //   return Math.abs(operatorJoystick.getRawAxis(0)) > 0.2;
    // }).onTrue(Commands.runOnce(new Runnable() {
    //   @Override
    //   public void run() {
    //     arm.setManualControl(true);
    //   }
    // }, arm)).whileTrue(Commands.run(new Runnable() {
    //   @Override
    //   public void run() {
    //     arm.setInput(operatorJoystick.getRawAxis(0));
    //   }
    // }, arm)).whileFalse(Commands.runOnce(new Runnable() {
    //   @Override
    //   public void run() {
    //     arm.setInput(0);
    //     arm.setManualControl(false);
    //   }
    // }));

    arm.setDefaultCommand(Commands.run(() -> {
      arm.setInput(MathUtil.applyDeadband(operatorJoystick.getRawAxis(0), 0.1));
    }, arm));
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
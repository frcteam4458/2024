package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveSubsystem;

public class PIDAngle extends PIDAlign {

    public PIDAngle(DriveSubsystem driveSubsystem, Rotation2d rotation, double ap, double ai, double ad) {
        super(driveSubsystem, new Pose2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY(), rotation), new BooleanSupplier() {
            public boolean getAsBoolean() { return false; }
        },
        0, 0, 0,
        ap, ai, ad);
    }
    
}

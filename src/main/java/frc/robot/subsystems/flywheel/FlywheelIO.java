// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;  

/** Add your docs here. */
public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        double topVoltage = 0.0;
        double bottomVoltage = 0.0;
        double topVelocity = 0.0;
        double bottomVelocity = 0.0;
        double topPosition = 0.0;
        double bottomPosition = 0.0;
        
        double topBusVoltage = 0.0;
        double bottomBusVoltage = 0.0;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {}
    public default void setTopSpeed(double speed) {}
    public default void setBottomSpeed(double speed) {}
    public default void setTopVoltage(double voltage) {}
    public default void setBottomVoltage(double voltage) {}
    public default double getVelocity() { return 0.0; }
    public default void updateSimulation() {}
}

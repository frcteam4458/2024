// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        double voltage = 0.0;
        double value = 0.0;
        double current = 0.0;
        double rpm = 0.0;
        double position = 0.0;
        boolean topSensor = false;
        boolean bottomSensor = false;
    }

    public default void updateInputs(FeederIOInputs inputs) {}

    public default void setVoltage(double volts) {}
    public default void set(double value) {}

}

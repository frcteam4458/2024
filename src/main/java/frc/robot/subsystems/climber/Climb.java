// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;

/** Add your docs here. */
public class Climb extends SubsystemBase {
    Solenoid solenoid;

    public Climb() {
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, HardwareConstants.kClimber);
    }

    public void set(boolean on) {
        solenoid.set(on);
    }

    public boolean get() {
        return solenoid.get();
    }

    public boolean toggle() {
        set(!get());
        return get();
    }
    
}

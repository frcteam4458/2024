// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;

/** Add your docs here. */
public class Climb extends SubsystemBase {
    DoubleSolenoid solenoid;

    public Climb() {
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void set(boolean on) {
        solenoid.set(on ? Value.kForward : Value.kReverse);
    }

    public boolean get() {
        if(solenoid.get().equals(Value.kForward)) return true;
        return false;
    }

    public boolean toggle() {
        set(!get());
        return get();
    }
    
}

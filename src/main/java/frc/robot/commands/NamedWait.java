// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Add your docs here. */
public class NamedWait extends WaitCommand {
    double duration = 0;

    public NamedWait() {
        super(0);
    }

    @Override
    public void initialize() {
        duration = SmartDashboard.getNumber("Named Wait Duration", duration);
        if(duration < 0) duration = 0;
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(duration);
    }
}

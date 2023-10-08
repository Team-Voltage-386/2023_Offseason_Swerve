// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Subsytems.Pneumatics;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Commands.ManipulatorCommands;

public class Robot extends TimedRobot {
    private Pneumatics m_Pneumatics = new Pneumatics();
    private final ManipulatorCommands m_manipulatorCommand = new ManipulatorCommands(m_Pneumatics);

    @Override
    public void robotInit()
    {
    }


    @Override
    public void teleopInit() {

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
        m_Pneumatics.controls();
    }

    @Override
    public void disabledPeriodic() {
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import Subsytems.Pneumatics;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ManipulatorCommands extends CommandBase {
  private Pneumatics PneumaticSystem;

  /** Creates a new ManipulatorCommands. */
  public ManipulatorCommands(Pneumatics pneumatics) {
    PneumaticSystem=pneumatics;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kManipulator.getRawButton(kRightBumper)) 
    {
      
    }
    else
    {
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
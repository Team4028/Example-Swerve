/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.climber;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.RobotContainer;
import com.swervedrivespecialties.exampleswerve.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class climb extends CommandBase {
  
  Climber _climber;

  public climb(Climber climber) {
    _climber = climber;
  }

  @Override
  public void initialize() {
    _climber.run(Robot.getRobotContainer().getSecondaryRightYAxis());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _climber.run(Robot.getRobotContainer().getSecondaryRightYAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _climber.run(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.climber;

import com.swervedrivespecialties.exampleswerve.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleClimbSolenoid extends CommandBase {
  
  Climber _climber;
  public ToggleClimbSolenoid(Climber climber) {
    _climber = climber;
  }

  @Override
  public void initialize() {
    _climber.toggleClimbSolenoid();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class YeetSingulator extends CommandBase {
  
  Infeed _infeed;
  CommandScheduler cs = CommandScheduler.getInstance();

  public YeetSingulator(Infeed infeed) {
    _infeed = infeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandBase cmd = YeetIntake.sCommand;
    if (cs.isScheduled(cmd)){
      cs.cancel(cmd);
    } else {
      cs.schedule(cmd);
    }

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

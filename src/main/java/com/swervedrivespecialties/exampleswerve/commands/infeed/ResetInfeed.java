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

public class ResetInfeed extends CommandBase {
  
  Infeed _infeed;

  public ResetInfeed(Infeed infeed) {
    _infeed = infeed;
  }


  @Override
  public void initialize() {
    _infeed.resetBallsConveyed();
    CommandScheduler.getInstance().cancel(YeetIntake.sCommand);
    CommandScheduler.getInstance().cancel(YeetIntake.ifCommand);
    _infeed.resetHasStoppedSingulating();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

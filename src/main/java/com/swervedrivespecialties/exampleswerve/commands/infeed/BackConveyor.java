/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BackConveyor extends CommandBase {

  Infeed _infeed;
  public BackConveyor(Infeed infeed) {
    _infeed = infeed;
  }

  @Override
  public void initialize() {
    _infeed.backConveyor();
  }

  @Override
  public void execute() {
    _infeed.backConveyor();
  }

  @Override
  public void end(boolean interrupted) {
    _infeed.stopConveyor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BumpConveyorForward extends CommandBase {
  
  Infeed _infeed;
  double targCount;

  public BumpConveyorForward(Infeed infeed) {
    _infeed = infeed;
  }

  @Override
  public void initialize() {
    targCount = _infeed.getConveyorPosiiton() + Infeed.kPostEyeEncoderTicks;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _infeed.conveyConveyor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _infeed.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _infeed.getConveyorPosiiton() > targCount;
  }
}

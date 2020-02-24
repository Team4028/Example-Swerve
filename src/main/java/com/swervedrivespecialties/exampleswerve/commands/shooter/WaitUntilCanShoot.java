/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;
import com.swervedrivespecialties.exampleswerve.util.util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilCanShoot extends CommandBase {
  
  private static final int kNumClearCycles = 3;
  private int numClearedCycles = 0;

  Shooter _shooter;
  public WaitUntilCanShoot(Shooter shooter) {
    _shooter = shooter;
  }

  @Override
  public void initialize() {
    numClearedCycles = 0;
  }

  @Override
  public void execute() {
    boolean clear = _shooter.getSpedUp();
    numClearedCycles = clear ? numClearedCycles + 1 : 0;
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return numClearedCycles > kNumClearCycles;
  }
}

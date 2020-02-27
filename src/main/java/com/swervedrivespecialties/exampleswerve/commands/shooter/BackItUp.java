/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BackItUp extends CommandBase {
  Shooter _shooter;
  Infeed _infeed;
  public BackItUp(Shooter shooter, Infeed infeed) {
    _shooter = shooter;
    _infeed = infeed;    
    addRequirements(_shooter);
  }

  @Override
  public void initialize() {
    _shooter.stopShooter();
    _shooter.stopKicker();
    _infeed.stopConveyor();
    _infeed.stopInfeed();
    _infeed.stopSingulator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _shooter.backKicker();
    _shooter.backShooter();
    _infeed.backConveyor();
    _infeed.backInfeed();
    _infeed.backSingulator();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.stopShooter();
    _shooter.stopKicker();
    _infeed.stopConveyor();
    _infeed.stopInfeed();
    _infeed.stopSingulator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

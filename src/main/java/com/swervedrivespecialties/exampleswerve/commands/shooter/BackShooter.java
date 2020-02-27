/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BackShooter extends CommandBase {
  
  Shooter _shooter;
  public BackShooter(Shooter shooter) {
    _shooter = shooter;
  }

  @Override
  public void initialize() {
    _shooter.backShooter();
  }

  @Override
  public void execute() {
    _shooter.backShooter();
  }

  @Override
  public void end(boolean interrupted) {
    _shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

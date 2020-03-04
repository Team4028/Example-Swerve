/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BangBang extends CommandBase {
  
  Shooter _shooter;
  double target;

  public BangBang(Shooter shooter, double targetSPD) {
    _shooter = shooter;
    target = targetSPD;
    addRequirements(_shooter);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bangIt();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.bangShooter(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void bangIt(){
    _shooter.bangShooter(_shooter.getSpeed() < target);
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetServo extends CommandBase {

  Shooter _shooter;

  public ResetServo(Shooter shooter) {
    _shooter = shooter;
  }

  @Override
  public void initialize() {
    _shooter.resetServo();
  }

  @Override 
  public boolean isFinished(){
    return true;
  }
}

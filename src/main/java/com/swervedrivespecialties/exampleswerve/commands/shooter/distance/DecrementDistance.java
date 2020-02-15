/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter.distance;

import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DecrementDistance extends InstantCommand {
  
  Shooter _shooter;

  public DecrementDistance(Shooter shooter) {
    _shooter = shooter;
  }

  @Override
  public void initialize() {
    _shooter.decrementShooterDistance();
  }
}

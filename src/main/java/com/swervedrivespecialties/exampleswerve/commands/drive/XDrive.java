/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class XDrive extends CommandBase {
  
  DrivetrainSubsystem _drive;

  public XDrive(DrivetrainSubsystem drive) {
    _drive = drive;
    addRequirements(_drive);
  }

  @Override
  public void initialize() {
    _drive.xDrive();
  }

  @Override
  public void execute() {
    _drive.xDrive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.RobotContainer;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AnalogBackKicker extends CommandBase {
  
  Shooter _shooter;
  public AnalogBackKicker(Shooter shooter) {
    _shooter = shooter;
    addRequirements(_shooter);
    }

  @Override
  public void initialize() {
    _shooter.stopKicker();
    _shooter.stopShooter();
  }

  @Override
  public void execute() {
    _shooter.analogRunKicker(Robot.getRobotContainer().getTertiaryRightYAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.stopKicker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

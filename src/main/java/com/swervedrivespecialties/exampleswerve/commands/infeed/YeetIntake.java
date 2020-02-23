/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import com.swervedrivespecialties.exampleswerve.commands.shooter.ShooterSubsystemCommands;
import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class YeetIntake extends CommandBase {

  Infeed _infeed;
  CommandScheduler cs = CommandScheduler.getInstance();

  public static CommandBase ifCommand = InfeedSubsystemCommands.getRunInfeedCommand();
  public static CommandBase sCommand = InfeedSubsystemCommands.getRunSingulatorCommand();

  //boolean isInfeedRunning

  public YeetIntake(Infeed infeed) {
    _infeed = infeed;
  }

  @Override
  public void initialize() {
    if (cs.isScheduled(ifCommand)){
      cs.cancel(ifCommand);
    } else {
      cs.schedule(ifCommand);
    }

    if (Infeed.get_instance().getCanSingulate()){
      if (!(cs.isScheduled(ifCommand) && !cs.isScheduled(sCommand))){
        cs.schedule(sCommand);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

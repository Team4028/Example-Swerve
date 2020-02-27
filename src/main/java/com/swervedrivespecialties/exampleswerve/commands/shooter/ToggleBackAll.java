/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.commands.infeed.InfeedSubsystemCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class ToggleBackAll extends InstantCommand {

  CommandScheduler cs = CommandScheduler.getInstance();
  CommandBase bi = InfeedSubsystemCommands.BACK_INFEED;
  CommandBase bsi = InfeedSubsystemCommands.BACK_SINGULATOR;
  CommandBase bc = InfeedSubsystemCommands.BACK_CONVEYOR;
  CommandBase bk = ShooterSubsystemCommands.BACK_KICKER;
  CommandBase bsh = ShooterSubsystemCommands.BACK_SHOOTER;

  public ToggleBackAll() {}

  @Override
  public void initialize() {
    if (cs.isScheduled(bi) &&
        cs.isScheduled(bsi) &&
        cs.isScheduled(bc) &&
        cs.isScheduled(bk) &&
        cs.isScheduled(bsh)){
          cs.cancel(bi);
          cs.cancel(bsi);
          cs.cancel(bc);
          cs.cancel(bk);
          cs.cancel(bsh);
        } else {
          cs.schedule(bi);
          cs.schedule(bsi);
          cs.schedule(bc);
          cs.schedule(bk);
          cs.schedule(bsh);
        }
  }
}

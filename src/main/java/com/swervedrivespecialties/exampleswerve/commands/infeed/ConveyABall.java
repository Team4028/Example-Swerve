/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConveyABall extends SequentialCommandGroup {
 
  
  public ConveyABall() {
    super(InfeedSubsystemCommands.getConveyCommand(), InfeedSubsystemCommands.getBackConveyorFixedComand());
  }
}

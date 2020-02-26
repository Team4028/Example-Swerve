/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;

public class runSingulator extends CommandBase {

  Infeed _infeed;

  public runSingulator(Infeed infeed) {
    _infeed = infeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!isFinished()){
      runSing();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isFinished()){
      runSing();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _infeed.stopSingulator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !_infeed.getCanSingulate();
  }

  private void runSing(){
    //!(_infeed.getPreConveyorSensor() && _infeed.getPostSingulatorSensor()) && 
    if (!isFinished()){
      _infeed.runSingulator();
    } else {
      _infeed.stopSingulator();
    }
  }
}

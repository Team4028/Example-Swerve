/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class convey extends CommandBase {
  private Infeed _infeed;
  double targetPos;
  boolean midConveyorSensorFreed = false;
  boolean midConveyorSensorPressed = false;

  private static final CommandBase backItUp = InfeedSubsystemCommands.getBackConveyorFixedComand();
  private static final CommandBase BringItForward = InfeedSubsystemCommands.getBumpConveyorForwardCommand();

  public convey(Infeed infeed) {
    _infeed = infeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPos = Infeed.get_instance().getConveyorPosiiton() + Infeed.kEncoderCountsPerBall;
    midConveyorSensorFreed = false;
    midConveyorSensorPressed = false;
    _infeed.startHandleConveyorLag();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _infeed.conveyConveyor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _infeed.stopConveyor();
    if (!interrupted){
      if (_infeed.getNumBallsConveyed() >= 3){
        backItUp.schedule();
      } else {
        if (_infeed.getNumBallsConveyed() == 2){
          BringItForward.schedule(true);
        } 
        _infeed.resetConveyorSensorPressed();
      }
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (_infeed.getPreShooterSensor()){
      return true;
    } else {
      if (_infeed.getHasFourthEye()){
        return updateMidConveyorPressed();
      } else {
        return _infeed.getConveyorPosiiton() > targetPos;
      }
    }
  }

  public boolean updateMidConveyorPressed(){
    if (!midConveyorSensorFreed){
      midConveyorSensorFreed = !_infeed.getMidConveyorSensor(); //If I haven't been freed, and I have no Mid Conveyor, then I'm free
      midConveyorSensorPressed = false;
    }

    if (midConveyorSensorFreed){
      midConveyorSensorPressed = _infeed.getMidConveyorSensor();
    }

    return midConveyorSensorPressed;
  }
}

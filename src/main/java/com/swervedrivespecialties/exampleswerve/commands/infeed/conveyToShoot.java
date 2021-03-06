/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


//Runs the conveyor to shoot
public class conveyToShoot extends CommandBase {
  private Infeed _infeed;

  double singulatorDelayTime = .4; //400 ms
  double startTime;


  public conveyToShoot(Infeed infeed) {
    _infeed = infeed;
    addRequirements(_infeed);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    _infeed.resetBallsConveyed();
    _infeed.resetHasStoppedSingulating();
    _infeed.stopSingulator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _infeed.conveyConveyorToShoot();
    if (Timer.getFPGATimestamp() - startTime > singulatorDelayTime){
      _infeed.runSingulatorToShoot();
      _infeed.runInfeed();
    } else {
      _infeed.stopSingulator();
      _infeed.stopInfeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _infeed.stopConveyor();
    _infeed.stopSingulator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

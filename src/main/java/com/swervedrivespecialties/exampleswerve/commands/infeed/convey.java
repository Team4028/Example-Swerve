/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class convey extends CommandBase {
  private Infeed _infeed;
  double targetPos;

  public convey(Infeed infeed) {
    _infeed = infeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPos = Infeed.get_instance().getConveyorPosiiton() + Infeed.kEncoderCountsPerBall;
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (_infeed.getConveyorPosiiton() > targetPos) || _infeed.getPreShooterSensor();
  }
}

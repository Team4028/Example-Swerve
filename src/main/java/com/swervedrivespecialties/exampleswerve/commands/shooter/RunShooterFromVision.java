/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.commands.infeed.YeetIntake;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class RunShooterFromVision extends CommandBase {
  Shooter _shooter;
  Limelight _ll = Limelight.getInstance();
  double _curTargVelo;
  double _actuatorVal;

  public RunShooterFromVision(Shooter shooter) {
    _shooter = shooter;
    addRequirements(_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shooter.isShooting = true;
    CommandScheduler.getInstance().cancel(YeetIntake.sCommand);
    CommandScheduler.getInstance().cancel(YeetIntake.ifCommand)
    ;
    run();
  }

  @Override 
  public void execute(){
    run();
  }

  @Override
  public boolean isFinished(){
    return false;
  }

  @Override 
  public void end(boolean interrupted){
    _shooter.isShooting = false;
    _shooter.runShooter(Shooter.Shot.getStopShot());
  }

  private void run(){
    _shooter.runShooter(_shooter.getShot());
  }
}

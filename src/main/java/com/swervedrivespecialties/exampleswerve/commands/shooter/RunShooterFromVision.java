/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.commands.infeed.YeetIntake;
import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;
import com.swervedrivespecialties.exampleswerve.util.BeakCircularBuffer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class RunShooterFromVision extends CommandBase {
  Shooter _shooter;
  Limelight _ll = Limelight.getInstance();
  double _curTargVelo;
  double _actuatorVal;

  double kReadyMaxDVelo = 220.6969696969696969; //nice
  int kMaxDVeloCycles = 7;
  int numCycles;

  BeakCircularBuffer speedBuffer = new BeakCircularBuffer(kMaxDVeloCycles);

  double lasttime;
  double thistime;
  double thisvelo;
  double lastvelo;



  public RunShooterFromVision(Shooter shooter) {
    _shooter = shooter;
    addRequirements(_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shooter.isShooting = true;
    _shooter.isSensorDistanceLocked = true;
    numCycles = 0;
    CommandScheduler.getInstance().cancel(YeetIntake.sCommand);
    CommandScheduler.getInstance().cancel(YeetIntake.ifCommand);
    Infeed.get_instance().resetHasStoppedSingulating();
    thisvelo = _shooter.getSpeed();
    thistime = Timer.getFPGATimestamp();
    run();
  }

  @Override 
  public void execute(){
    numCycles++;
    updateDVelos();
    run();
    _shooter.updateCanShoot(getSpedUp());
  }

  @Override
  public boolean isFinished(){
    return false;
  }

  @Override 
  public void end(boolean interrupted){
    _shooter.isShooting = false;
    //_shooter.isSensorDistanceLocked = false;
    _shooter.updateCanShoot(false);
    _shooter.runShooter(Shooter.Shot.getStopShot());
  }

  private void run(){
    _shooter.runShooter(_shooter.getShot());
  }

  private void updateDVelos(){
    lasttime = thistime;
    thistime = Timer.getFPGATimestamp();
    double dt = thistime - lasttime;
    lastvelo = thisvelo;
    thisvelo = _shooter.getSpeed();
    double dv = thisvelo - lastvelo;
    double accel = Math.abs(dv / dt);
    speedBuffer.addFirst(accel);
  }

  private boolean getSpedUp(){
    return speedBuffer.getMean() < kReadyMaxDVelo && numCycles > kMaxDVeloCycles;
  }
}

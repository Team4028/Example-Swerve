/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.util.util;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LLRotateToTarget extends CommandBase {
  
  DrivetrainSubsystem _drive;
  Limelight _limelight = Limelight.getInstance();


  PidController _rotController = new PidController(new PidConstants(0.013, 0.0125, 0.001)); //0.013, 0.0125, 0.0008

  private double _prevTime,  error, forward, strafe;
  private boolean _translate;

  boolean shouldStopFirstCycle;
  boolean isFirstCycle;

  public LLRotateToTarget(DrivetrainSubsystem drive, boolean canTranslate) {
    _drive = drive;
    _translate = canTranslate;
    _rotController.setContinuous(true);
    _rotController.setInputRange(-180, 180);
    _rotController.setOutputRange(-1, 1);
    _rotController.setSetpoint(0);
    _rotController.setIntegralRange(10);
  }

  public LLRotateToTarget(DrivetrainSubsystem drive){
    this(drive, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _prevTime = Timer.getFPGATimestamp();
    error = _limelight.accountForSkew(_limelight.getAngle1());
    isFirstCycle = true;
    shouldStopFirstCycle = !_limelight.getHasTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isFirstCycle){
      isFirstCycle = false;
    }
    double dt = Timer.getFPGATimestamp() - _prevTime;
    _prevTime = Timer.getFPGATimestamp();
    
    error = _limelight.accountForSkew(_limelight.getAngle1());
    double rot = _rotController.calculate(error, dt);
    _drive.drive(_drive.getDriveVec().times(util.iversonBrackets(_translate)), rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isFirstCycle && shouldStopFirstCycle){
      System.out.println("TRIED TO AIM WITHOUT VISION!!! STOPPING!!!");
      return shouldStopFirstCycle;
    }
    return Math.abs(_limelight.accountForSkew(_limelight.getAngle1())) <= 0.3;
  }
}

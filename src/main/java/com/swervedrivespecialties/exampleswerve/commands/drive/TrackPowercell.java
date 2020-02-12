/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.Chameleon;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TrackPowercell extends CommandBase {
  
  DrivetrainSubsystem _drive;
  Chameleon _chameleon;


  PidController _rotController = new PidController(new PidConstants(0.012, 0.01, 0.0008));

  private double _prevTime, error, prevError, prevGyro;

  public TrackPowercell(Chameleon chameleon, DrivetrainSubsystem subsystem) {
    _chameleon = chameleon;
    _drive = subsystem;
    _rotController.setContinuous(true);
    _rotController.setInputRange(-180, 180);
    _rotController.setOutputRange(-1, 1);
    _rotController.setSetpoint(7.3);
    _rotController.setIntegralRange(10);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _prevTime = Timer.getFPGATimestamp();
    error = _chameleon.getAngle1();
    prevGyro = _drive.getGyroAngle().toDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dt = Timer.getFPGATimestamp() - _prevTime;
    _prevTime = Timer.getFPGATimestamp();
    double minSS = DrivetrainSubsystem.getInstance().getMinControllerSpeed();
    double additionalSS =  Robot.getRobotContainer().getPrimaryRightTrigger();
    double speedScale = minSS + (1 - minSS) * additionalSS * additionalSS;
    if (_chameleon.getAngle1() != prevError){
      error = _chameleon.getAngle1();
      prevGyro = _drive.getGyroAngle().toDegrees();
      prevError = error;
    } else{
      error = prevError - (_drive.getGyroAngle().toDegrees() - prevGyro);
      prevGyro = _drive.getGyroAngle().toDegrees();
    }
    double forward = -Robot.getRobotContainer().getPrimaryLeftYAxis();
    forward = Utilities.deadband(forward);
    // Square the forward stick
    forward = speedScale * Math.copySign(Math.pow(forward, 2.0), forward);

    double strafe = -Robot.getRobotContainer().getPrimaryLeftXAxis();
    strafe = Utilities.deadband(strafe);
    // Square the strafe stick
    strafe = speedScale * Math.copySign(Math.pow(strafe, 2.0), strafe);

    double rot = _rotController.calculate(error, dt);

    _drive.drive(new Translation2d(0, 0), rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error - 7.3) <= 0.5;
  }
}

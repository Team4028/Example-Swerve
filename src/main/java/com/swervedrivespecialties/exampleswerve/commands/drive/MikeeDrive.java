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
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.CAMERA_SLOT;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MikeeDrive extends CommandBase {
  DrivetrainSubsystem _drive;

  double kRot = .025;
  double kDeadBand = .06; //custom because of the unique demands of this task

  boolean shouldFinish = false;
  
  public MikeeDrive(DrivetrainSubsystem drive) {
    _drive = drive;
    addRequirements(_drive);
  }

  @Override
  public void initialize() {
    Limelight.getInstance().setCameraSlot(CAMERA_SLOT.LIMELIGHT);
  }

  @Override
  public void execute() {
    double rotVal = -Robot.getRobotContainer().getPrimaryRightXAxis();
    _drive.drive(new Translation2d(), Math.abs(rotVal) > kDeadBand ? Math.copySign(kRot, rotVal) : 0.0, true);
  }

  @Override
  public void end(boolean interrupted) {
    _drive.drive(new Translation2d(), 0, true);
    Limelight.getInstance().setCameraSlot(CAMERA_SLOT.INFEED);
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}
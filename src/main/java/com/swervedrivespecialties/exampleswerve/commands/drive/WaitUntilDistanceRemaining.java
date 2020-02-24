/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilDistanceRemaining extends CommandBase {
  
  DrivetrainSubsystem _drive;
  Supplier<Trajectory> trajSup;
  double distance;
  Vector2 target;

  public WaitUntilDistanceRemaining(DrivetrainSubsystem drive, Supplier<Trajectory> tSupplier, double dist) {
    _drive = drive;
    distance = dist;
    trajSup = tSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Trajectory t = trajSup.get();
    target = t.calculateSegment(t.getDuration()).translation;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Vector2 diff = target.subtract(_drive.getKinematicPosition());
    return Math.sqrt(diff.dot(diff)) < distance;
  }
}

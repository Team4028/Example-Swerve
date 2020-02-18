/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class printTargetToLL extends CommandBase {
  /**
   * Creates a new printTargetToLL.
   */
  public printTargetToLL() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(Limelight.getInstance().getTargetToLL().plus(new Translation2d(29.5, 0)).toString());
    System.out.println(Double.toString(Limelight.getInstance().locateFlavortownUSA().toDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLLLEDMode extends CommandBase {
  /**
   * Creates a new SetLLLEDMode.
   */
  private double mode;
  public SetLLLEDMode() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mode = Limelight.getInstance().getLedMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mode = Limelight.getInstance().getLedMode() == 0.0 ? 1:0;
    Limelight.getInstance().setLedMode(mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.auton.autons.opponentball;

import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
import com.swervedrivespecialties.exampleswerve.commands.drive.DriveSubsystemCommands;
import com.swervedrivespecialties.exampleswerve.commands.infeed.InfeedSubsystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class HereWeGoAgain extends SequentialCommandGroup {

  
  public HereWeGoAgain() {
    super(
      new OpponentBallAuton(), 
      DriveSubsystemCommands.getRotateToAngleCommand(Trajectories.steallBallAuton.towardsBallsRotation.toDegrees(), 2),
      new ParallelCommandGroup(
        DriveSubsystemCommands.getFollowTrajectoryCommand(Trajectories.steallBallAuton.toPickupNextTrajectorySupplier),
        new SequentialCommandGroup(
          new WaitCommand(.4),
          InfeedSubsystemCommands.getRunInfeedCommand().withTimeout(3.5)
        )
      )
    );
  }
}

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
import com.swervedrivespecialties.exampleswerve.commands.shooter.ShooterSubsystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CouldItBeSeven extends SequentialCommandGroup {

  public CouldItBeSeven() {
    super(
      new OpponentBallBestAuton(), 
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            DriveSubsystemCommands.getFollowTrajectoryCommand(Trajectories.sevenBallAuton.toGetNextBatchTrajectorySupplier),
            new SequentialCommandGroup(
              new WaitCommand(.5), 
              InfeedSubsystemCommands.getRunInfeedCommand().withTimeout(1.2)
            )
          ),
          DriveSubsystemCommands.getFollowTrajectoryCommand(Trajectories.sevenBallAuton.toShootNextShotTrajectorySupplier), 
          DriveSubsystemCommands.getRotateToAngleCommand(140)
        ), 
      InfeedSubsystemCommands.getRunSingulatorCommand()
      ), 
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          ShooterSubsystemCommands.getWaitUntilCanShootCommand(), 
          InfeedSubsystemCommands.getConveyToShootCommand().withTimeout(2)
        ),
        DriveSubsystemCommands.getLLRotateToTargetCommand(), 
        ShooterSubsystemCommands.getRunShooterFromVisionCommand()
      )      
    );
  }
}

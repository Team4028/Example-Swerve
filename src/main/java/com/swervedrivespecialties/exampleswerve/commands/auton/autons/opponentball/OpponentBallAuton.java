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
import com.swervedrivespecialties.exampleswerve.util.InertiaGain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OpponentBallAuton extends SequentialCommandGroup {
  
  public OpponentBallAuton() {
    super(
      new ParallelCommandGroup(
        DriveSubsystemCommands.getFollowTrajectoryCommand(Trajectories.steallBallAuton.toStealBallsTrajectorySupplier,
                                                          new InertiaGain(0, 0, 0)), 
        new SequentialCommandGroup(
          new WaitCommand(1.5)
          , InfeedSubsystemCommands.getAutonInfeedCommand(2, 3)
        )
      ),
      new ParallelCommandGroup(
        DriveSubsystemCommands.getFollowTrajectoryCommand(Trajectories.steallBallAuton.toShootFirstBatchTrajectorySupplier,
                                                          new InertiaGain(0, 0, 0)),
        new SequentialCommandGroup(
          new WaitCommand(3),
          ShooterSubsystemCommands.getRunShooterFromVisionCommand()
        )
      ),
      InfeedSubsystemCommands.getConveyToShootCommand().withTimeout(2)   
    );
  }
}
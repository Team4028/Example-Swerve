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

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OpponentBallAuton extends ParallelDeadlineGroup {
  public OpponentBallAuton() {
    super(
          new SequentialCommandGroup(
                                      new ParallelCommandGroup(
                                                                DriveSubsystemCommands.getFollowTrajectoryCommand(Trajectories.steallBallAuton.toStealBallsTrajectorySupplier,
                                                                                                                  new InertiaGain(0, 0, 0))
                                                              ),
                                      DriveSubsystemCommands.getFollowTrajectoryCommand(Trajectories.steallBallAuton.toShootFirstBatchTrajectorySupplier),
                                      new ParallelRaceGroup(
                                        ShooterSubsystemCommands.getRunShooterFromVisionCommand().withTimeout(8), 
                                        new SequentialCommandGroup(
                                                                    ShooterSubsystemCommands.getWaitUntilCanShootCommand(),
                                                                    InfeedSubsystemCommands.getConveyToShootCommand().withTimeout(3.5)
                                        ) 
                                      )
          ), 
          InfeedSubsystemCommands.getRunSingulatorCommand(), 
          new SequentialCommandGroup(
            new WaitCommand(.5),
            InfeedSubsystemCommands.getRunInfeedCommand().withTimeout(14)                                                                                                                         
          )
        );
  }
}
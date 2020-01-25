/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
import com.swervedrivespecialties.exampleswerve.commands.drive.DriveCommand;
import com.swervedrivespecialties.exampleswerve.commands.drive.DriveSubsystemCommands;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance();

    private Joystick primaryJoystick = new Joystick(0);
    private Joystick secondaryJoystick = new Joystick(1);

    private void bindPrimaryJoystickButtons(){
        // final JoystickButton primary_a = new JoystickButton(primaryJoystick, 1);
        // final JoystickButton primary_b = new JoystickButton(primaryJoystick, 2);
        // final JoystickButton primary_x = new JoystickButton(primaryJoystick, 3);
        final JoystickButton primary_y = new JoystickButton(primaryJoystick, 4);
        final JoystickButton primary_left_bumper = new JoystickButton(primaryJoystick, 5);
        final JoystickButton primary_right_bumper = new JoystickButton(primaryJoystick, 6);
        final JoystickButton primary_back = new JoystickButton(primaryJoystick, 7);
        final JoystickButton primary_start = new JoystickButton(primaryJoystick, 8);

        primary_back.whenPressed(DriveSubsystemCommands.getZeroGyroCommand());
        primary_start.whenPressed(DriveSubsystemCommands.getToggleFieldOrientedCommand());
        primary_y.whenPressed(DriveSubsystemCommands.getToggleSpeedCommand());
        primary_right_bumper.whenPressed(DriveSubsystemCommands.getFollowTrajectoryCommand(Trajectories.testTrajectorySupplier));
        primary_left_bumper.whenPressed(DriveSubsystemCommands.getRotateToAngleCommand(180));
    }

    private void bindSecondaryJoystickButtons(){
        // final JoystickButton secondary_a = new JoystickButton(primaryJoystick, 1);
        // final JoystickButton secondary_b = new JoystickButton(primaryJoystick, 2);
        // final JoystickButton secondary_x = new JoystickButton(primaryJoystick, 3);
        // final JoystickButton secondary_y = new JoystickButton(primaryJoystick, 4);
        // final JoystickButton secondary_left_bumper = new JoystickButton(primaryJoystick, 5);
        // final JoystickButton secondary_right_bumber= new JoystickButton(primaryJoystick, 6);
        // final JoystickButton secondary_back = new JoystickButton(primaryJoystick, 7);
        // final JoystickButton secondary_start = new JoystickButton(primaryJoystick, 8);
    }

    public RobotContainer(){
        bindPrimaryJoystickButtons();
        bindSecondaryJoystickButtons();
        initDefaultCommands();
    }

    public double getPrimaryLeftXAxis(){
        return primaryJoystick.getRawAxis(0);
    }

    public double getPrimaryLeftYAxis(){
        return primaryJoystick.getRawAxis(1);
    }

    public double getPrimaryLeftTrigger(){
        return primaryJoystick.getRawAxis(2);
    }

    public double getPrimaryRightTrigger(){
        return primaryJoystick.getRawAxis(3);
    }

    public double getPrimaryRightXAxis(){
        return primaryJoystick.getRawAxis(4);
    }

    public double getPrimaryRightYAxis(){
        return primaryJoystick.getRawAxis(5);
    }

    public void initDefaultCommands(){
        CommandScheduler.getInstance().setDefaultCommand(drive, DriveSubsystemCommands.getDriveCommand());
    }
}
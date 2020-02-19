/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.commands.drive.DriveSubsystemCommands;
import com.swervedrivespecialties.exampleswerve.commands.drive.printTargetToLL;
import com.swervedrivespecialties.exampleswerve.commands.shooter.ShooterSubsystemCommands;
import com.swervedrivespecialties.exampleswerve.commands.infeed.InfeedSubsystemCommands;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;
import com.swervedrivespecialties.exampleswerve.util.BeakXBoxController;
import com.swervedrivespecialties.exampleswerve.util.DataLogger;
import com.swervedrivespecialties.exampleswerve.util.LogDataBE;
import com.swervedrivespecialties.exampleswerve.util.util;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {

    private DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private Limelight limelight = Limelight.getInstance();
    private Infeed infeed = Infeed.get_instance();

    private BeakXBoxController primary = new BeakXBoxController(0);
    private BeakXBoxController secondary = new BeakXBoxController(1);

    private DataLogger _dataLogger = null;

    private void bindPrimaryJoystickButtons(){
        //primary.a.toggleWhenPressed(InfeedSubsystemCommands.getConveyToShootCommand());
        //primary.b.toggleWhenPressed(ShooterSubsystemCommands.getRunShooterFromVisionCommand());
        primary.x.whenPressed(DriveSubsystemCommands.getRotateAboutTheCenterOfTheRobotToPointTowardsFlavortown());
        primary.y.whenPressed(DriveSubsystemCommands.getToggleSpeedCommand());
        primary.left_bumper.toggleWhenPressed(DriveSubsystemCommands.getMikeeDriveCommand());
       // primary.right_bumper.whenPressed(DriveSubsystemCommands.getFollowTrajectoryCommand(Trajectories.testTrajectorySupplier));
        primary.back.whenPressed(DriveSubsystemCommands.getZeroGyroCommand());
        primary.start.whenPressed(DriveSubsystemCommands.getToggleFieldOrientedCommand());
        primary.a.whenPressed(DriveSubsystemCommands.getToggleLEDMode());
    }

    private void bindSecondaryJoystickButtons(){
       secondary.a.toggleWhenPressed(InfeedSubsystemCommands.getConveyToShootCommand());
       secondary.y.whenPressed(InfeedSubsystemCommands.getRunSingulatorCommand());
       secondary.x.toggleWhenPressed(InfeedSubsystemCommands.getRunInfeedCommand());
       secondary.b.toggleWhenPressed(ShooterSubsystemCommands.getRunShooterFromVisionCommand());
       secondary.left_bumper.whenPressed(InfeedSubsystemCommands.getToggleInfeedSolenoidCommand());
       secondary.back.whenPressed(ShooterSubsystemCommands.getResetServoCommand());
       secondary.dpad_left.whenPressed(ShooterSubsystemCommands.getResetDistanceCommand());
       secondary.dpad_right.whenPressed(ShooterSubsystemCommands.getClearDistanceOffsetCommand());
       secondary.dpad_up.whenPressed(ShooterSubsystemCommands.getIncremenetDistanceCommand());
       secondary.dpad_down.whenPressed(ShooterSubsystemCommands.getDecremenetDistanceCommand());
       secondary.start.whenPressed(ShooterSubsystemCommands.getTogggleAlternateShotCommand());
    }

    public RobotContainer(){
        bindPrimaryJoystickButtons();
        bindSecondaryJoystickButtons();
        initDefaultCommands();
    }

    public double getPrimaryLeftXAxis(){
        return primary.getLeftXAxis();
    }

    public double getPrimaryLeftYAxis(){
        return primary.getLeftYAxis();
    }

    public double getPrimaryLeftTrigger(){
        return primary.getLeftTrigger();
    }

    public double getPrimaryRightTrigger(){
        return primary.getRightTrigger();
    }

    public double getPrimaryRightXAxis(){
        return primary.getRighXAxis();
    }

    public double getPrimaryRightYAxis(){
        return primary.getRightYAxis();
    }

    public void initDefaultCommands(){
        CommandScheduler.getInstance().setDefaultCommand(drive, DriveSubsystemCommands.getDriveCommand());
    }

    public void setupLogging(boolean auto){
        if (auto){
            _dataLogger = util.setupLogging("Auton");
        } else {
            _dataLogger = util.setupLogging("Teleop");
        }
    }

    public void logAllData(){
        if(_dataLogger != null) {    	
	    	// create a new, empty logging class
            LogDataBE logData = new LogDataBE();
            if (drive != null){ drive.updateLogData(logData); }
            if (limelight != null){ limelight.updateLogData(logData); }
            if (shooter != null ){ shooter.updateLogData(logData); }

            _dataLogger.WriteDataLine(logData);
        }
    }

    public void configureDrive(){
        drive.reset();
        drive.setCurrentLimit(40);
    }

    public static void configureInfeed(){
        Infeed.get_instance().setSolenoidOut(true);
    }

    public void outputToSDB(){
        shooter.outputToSDB();
        infeed.outputToSDB();
    }
}

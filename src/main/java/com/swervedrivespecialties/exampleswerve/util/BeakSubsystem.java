/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Add your docs here.
 */
public abstract class BeakSubsystem implements Subsystem{
    public abstract void config(boolean isAuton); //config will run once on init, in auto if true or in teleop if false
    public abstract void outputToSDB(); //this is run periodically updating SmartDashboard. Runs whether disabled or not
    public abstract void updateLogData(LogDataBE logData); //optionally logs all data. Only runs when enabled
    public abstract void stop(); //this is run in the case of problems to stop the robot. 
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BeakSubsystem extends SubsystemBase {
    public abstract void outputToSDB();
    public abstract void updateLogData(LogDataBE logData);
    public abstract void init(boolean isAuto);
    public abstract void stop();
}

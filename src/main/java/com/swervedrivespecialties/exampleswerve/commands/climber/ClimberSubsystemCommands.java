/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.climber;

import com.swervedrivespecialties.exampleswerve.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberSubsystemCommands {
    public static Climber climber = Climber.getInstance();

    public static CommandBase getClimbCommand(){
        return new climb(climber);
    }

    public static CommandBase getToggleClimbSolenoidCommand(){
        return new ToggleClimbSolenoid(climber);
    }
}

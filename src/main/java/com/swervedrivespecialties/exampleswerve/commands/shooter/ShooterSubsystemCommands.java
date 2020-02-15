/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.commands.shooter.distance.*;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterSubsystemCommands {
    public static Shooter shooter = Shooter.getInstance();

    public static CommandBase getRunShooterFromVisionCommand(){
        return new RunShooterFromVision(shooter);
    }

    public static CommandBase getResetServoCommand(){
        return new ResetServo(shooter);
    }

    public static CommandBase getTogggleAlternateShotCommand(){
        return new ToggleAlternateShot(shooter);
    }    

    public static CommandBase getIncremenetDistanceCommand(){
        return new IncrementDistance(shooter);
    }

    public static CommandBase getDecremenetDistanceCommand(){
        return new DecrementDistance(shooter);
    }

    public static CommandBase getResetDistanceCommand(){
        return new ResetDistance(shooter);
    }

    public static CommandBase getClearDistanceOffsetCommand(){
        return new ClearDistanceOffset(shooter);
    }
}
